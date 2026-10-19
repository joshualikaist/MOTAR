"""07 §§1,11–12 [NEW]: canonical online-only SHA-256 and independent trace copies.

The serialization uses framed ASCII float.hex values, dtype and shape, independent
of native byte order. Post-reset rows are omitted BEFORE input serialization.
Pure SHA-256 avoids an unapproved package dependency; tests use hashlib as oracle.
"""
import math
from dataclasses import fields

import torch


def _primes(count):
    result, candidate = [], 2
    while len(result) < count:
        if all(candidate % p for p in result if p <= math.isqrt(candidate)):
            result.append(candidate)
        candidate += 1
    return tuple(result)


def _cube_root(value):
    lower, upper = 0, 1 << ((value.bit_length() + 2) // 3)
    while lower + 1 < upper:
        middle = (lower + upper) // 2
        if middle ** 3 <= value:
            lower = middle
        else:
            upper = middle
    return lower


_PRIMES = _primes(64)
_INITIAL = tuple(math.isqrt(p << 64) & 0xffffffff for p in _PRIMES[:8])
_ROUND = tuple(_cube_root(p << 96) & 0xffffffff for p in _PRIMES)


def sha256(payload):
    """Bytes -> standard 32-byte SHA-256; immutable constants and local work only."""
    if not isinstance(payload, bytes) or len(payload) >= 1 << 61:
        raise ValueError("SHA input must be bounded bytes")
    size = len(payload)
    padded = payload + b"\x80" + b"\0" * ((55 - size) % 64) + (8 * size).to_bytes(8, "big")
    state = list(_INITIAL)
    rotate = lambda x, n: (x >> n) | ((x << (32 - n)) & 0xffffffff)
    for start in range(0, len(padded), 64):
        words = [int.from_bytes(padded[start + j:start + j + 4], "big") for j in range(0, 64, 4)]
        for j in range(16, 64):
            x, y = words[j - 15], words[j - 2]
            s0 = rotate(x, 7) ^ rotate(x, 18) ^ (x >> 3)
            s1 = rotate(y, 17) ^ rotate(y, 19) ^ (y >> 10)
            words.append((words[j - 16] + s0 + words[j - 7] + s1) & 0xffffffff)
        a, b, c, d, e, f, g, h = state
        for constant, word in zip(_ROUND, words):
            s1 = rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25)
            t1 = (h + s1 + ((e & f) ^ (~e & g)) + constant + word) & 0xffffffff
            s0 = rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22)
            t2 = (s0 + ((a & b) ^ (a & c) ^ (b & c))) & 0xffffffff
            a, b, c, d, e, f, g, h = (t1 + t2) & 0xffffffff, a, b, c, (d + t1) & 0xffffffff, e, f, g
        state = [(x + y) & 0xffffffff for x, y in zip(state, (a, b, c, d, e, f, g, h))]
    return b"".join(x.to_bytes(4, "big") for x in state)


def _frame(payload):
    return len(payload).to_bytes(8, "big") + payload


def _scalar(value):
    if isinstance(value, tuple):
        return b"tuple" + b"".join(_frame(_scalar(x)) for x in value)
    if isinstance(value, float):
        return b"float:" + value.hex().encode("ascii")
    return (type(value).__name__ + ":" + str(value)).encode("ascii")


def canonical_input(inp, env):
    """One permitted OnlineInputs row -> versioned canonical bytes, no task access."""
    result = b"BOIA_INPUT_V1\0"
    for field in fields(inp):
        value = getattr(inp, field.name)
        result += _frame(field.name.encode("ascii"))
        if isinstance(value, torch.Tensor):
            row = value if field.name in ("angles", "elevations") else value[env]
            values = b"".join(_frame(_scalar(x)) for x in row.detach().cpu().reshape(-1).tolist())
            payload = _frame(str(row.dtype).encode("ascii")) + _frame(str(tuple(row.shape)).encode("ascii")) + values
        elif field.name == "cam":
            payload = b"".join(_frame(f.name.encode("ascii")) + _frame(_scalar(getattr(value, f.name))) for f in fields(value))
        else:
            payload = _scalar(value)
        result += _frame(payload)
    return result


def record(decision, inp, terminal_prev):
    """Return dict of copied NumPy arrays, containing live rows only.

    None input is permitted only when all rows are terminal. No fabricated digest
    stands in for a post-reset observation. Costs/advantages may legitimately be inf.
    """
    ids = (~terminal_prev).nonzero(as_tuple=False).flatten()
    if inp is None:
        assert ids.numel() == 0
        hashes = torch.empty((0, 8), dtype=torch.uint8)
    else:
        hashes = torch.tensor([list(sha256(canonical_input(inp, e))[:8]) for e in ids.cpu().tolist()],
                              dtype=torch.uint8).reshape(-1, 8)
    export = lambda t: t.detach().cpu().numpy().copy()
    output = {"env_index": export(ids), "input_digest": export(hashes)}
    for name in ("candidate_commands", "hard_ok", "stop_ok", "cost", "advantage", "mode", "engaged", "reason", "action"):
        output[name] = export(getattr(decision, name)[ids])
    for name, value in decision.terms.items():
        output["term_" + name] = export(value[ids])
    if inp is not None:
        output["policy_action"] = export(inp.policy_action[ids])
        output["action_changed"] = export((decision.action[ids] != inp.policy_action[ids]).any(-1))
    return output
