#!/usr/bin/env python3
"""Offline real-frame reflection audit for the frozen NavRL ref5in policy (N1).

Preregistration: ``docs/prereg_2026-08-21_n1_real_frame_reflection_audit.md`` (frozen).
Every threshold, statistic and context split below is fixed by that document.

What this tool is
-----------------
An OFFLINE evaluator.  It loads (a) a ``.npz`` of real 898-D actor observations dumped during a
standard evaluation rollout and (b) the frozen checkpoint, then runs deterministic forward passes
on each observation and on its exact left-right reflection.  It NEVER creates a simulator, never
imports Isaac Gym, never steps an environment and never feeds a probe action anywhere.  The
temporal separation between rollout and this program is itself the proof of the "side-forward
only" condition (prereg section 4).

Numerical fidelity to the live player
-------------------------------------
``aerial_gym/rl_training/rl_games/navrl_players.py`` (``NavRLPpoPlayerContinuous.get_action``)
does, for the reflection diagnostic:

    original_obs = self._preproc_obs(obs)
    mirrored_obs = self._preproc_obs(mirror_navrl_structured_observation(obs))   # MIRROR FIRST
    ... self.model(...) ...                       # running_mean_std lives INSIDE the model
    action = self._model_action(res_dict, True)   # deterministic_actions, else mus

This tool instantiates the REAL ``NavRLPpoPlayerContinuous`` and calls the REAL ``.restore()``.
No network is hand-rolled and no normaliser is recomputed.  A vecenv is avoided by injecting
``env_info`` into the player config (``rl_games.common.player.BasePlayer.__init__`` only builds an
environment when ``config['env_info']`` is absent), so ``.run()`` is never called and no simulator
exists at any point.

Order matters and is preserved: the observation is mirrored in RAW units and only then handed to
the model, whose ``norm_obs`` applies the checkpoint's ``running_mean_std``.  That order is
physically correct -- a reflected world produces raw observations that pass through the same fixed
normaliser -- and is deliberately NOT commuted (prereg L2).

``deterministic_actions`` (post-tanh, in [-1, 1]) is preferred over ``mus``; for this squashed
Gaussian policy ``mus`` is the PRE-tanh latent mean and is the wrong quantity.  Actions are read
BEFORE any rescale/clamp to ``actions_low``/``actions_high``, i.e. exactly the units of
``navrl_players._record_reflection_pair``.

Usage
-----
    PYTHONNOUSERSITE=1 python tools/navrl_reflection_offline_audit.py \
        --frames results/.../obs_dump.npz \
        --checkpoint <...>/last_gen_ppo_ep_1900_rew_182.11377.pth \
        --checkpoint-sha256 197ea269... \
        --out results/.../reflection_offline_audit.json

Exit codes: 0 = audit completed (any verdict), 2 = precondition failure (bad SHA, bad schema,
unsupported checkpoint), 3 = a preregistered quality gate failed (fail-closed).
"""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import types

import numpy as np


# --------------------------------------------------------------------------------------------
# PREREGISTERED CONSTANTS.  Declared above every measurement so that no measurement code can
# introduce a second, drifting copy of a threshold.  Prereg sections 5, 6 and 7.
# --------------------------------------------------------------------------------------------

SCHEMA_VERSION = 1
PRODUCER = "tools/navrl_reflection_offline_audit.py"

# Prereg section 7 -- verdict thresholds, applied to the OVERALL cell only.
VERDICT_CONFIRM_MEDIAN_MIN = 0.30
VERDICT_CONFIRM_SIGN_AGREEMENT_MAX = 0.60
VERDICT_ABSENT_MEDIAN_MAX = 0.10
VERDICT_ABSENT_SIGN_AGREEMENT_MIN = 0.90

VERDICT_CHIRALITY_CONFIRMED = "CHIRALITY_CONFIRMED_REAL_FRAME"
VERDICT_CHIRALITY_ABSENT = "CHIRALITY_ABSENT"
VERDICT_INCONCLUSIVE = "INCONCLUSIVE_REAL_FRAME"
VERDICT_FAIL_CLOSED = "FAIL_CLOSED_TRANSFORM_QUALITY"

# Prereg section 6 -- "meaningful sign" floor, identical to navrl_players._record_reflection_pair.
LATERAL_SIGN_THRESHOLD = 0.05
# Prereg section 6 -- a context cell needs this many comparable rows to receive a verdict.
MIN_CONTEXT_COMPARABLE_ROWS = 256
# Prereg section 5 (Q9) -- minimum valid frames.
MIN_VALID_FRAMES = 4096

# Prereg section 5 -- quality-gate thresholds.
GATE_INVOLUTION_MAX_ABS = 0.0
GATE_ISOMETRY_MAX_ABS = 1.0e-3
GATE_STRUCTURED_OBS_DIM = 898
GATE_SCAN_FIXED_POINTS = frozenset((0, 36))

# Prereg section 5 -- structured schema block layout (the field widths are fixed by
# navrl_perception.py; the beam/token counts come from the checkpoint metadata).
OBSTACLE_HISTORY = 5
OBSTACLE_DIM = 12
OBSTACLE_SIGN_FLIP_FIELDS = (1, 4)
ROBOT_HISTORY = 5
ROBOT_DIM = 10
ROBOT_SIGN_FLIP_FIELDS = (1, 3, 5, 7)
TARGET_HISTORY = 5
TARGET_DIM = 16
TARGET_SIGN_FLIP_FIELDS = (1, 4)

# Prereg section 6 -- reported order statistics and the percentile convention.
PERCENTILE_QUANTILES = (90.0, 95.0, 99.0)
PERCENTILE_METHOD = "linear"
PERCENTILE_CONVENTION = (
    "numpy.percentile(values, q, method='linear') on float64; median is q=50 under the same "
    "convention"
)

# Prereg section 4 -- outcome attribution codes emitted by the NAVRL_OBS_DUMP hook.  Literal on
# purpose: a computed map could silently re-label a stratum.
OUTCOME_CODES = {
    0: "capture",
    1: "crash_bar_contact",
    2: "crash_oob",
    3: "crash_other",
    4: "timeout",
    5: "unattributed",
}
# Prereg section 6 -- the five preregistered outcome context cells (5 = unattributed is NOT a
# context; its frame count is reported as diagnostics only).
OUTCOME_CONTEXT_CODES = (0, 1, 2, 3, 4)

# Action channel indices (prereg section 6): x, lateral, z, yaw.
ACTION_CHANNELS = (("conj_err_x", 0), ("conj_err_lat", 1), ("conj_err_z", 2), ("conj_err_yaw", 3))
LATERAL_ACTION_INDEX = 1

DEFAULT_BATCH_SIZE = 1024
CHECKPOINT_ENV_KEYS = (
    "cfg_lidar_hbeams",
    "cfg_lidar_vbeams",
    "cfg_max_obstacles",
    "cfg_corridor_tokens",
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAIN_CONFIG = ROOT / "aerial_gym/rl_training/rl_games/ppo_navrl_perception_transformer.yaml"


class AuditPreconditionError(RuntimeError):
    """A precondition that makes the audit meaningless (bad SHA, unsupported schema)."""


class GateFailure(RuntimeError):
    """One or more preregistered quality gates failed; no policy claim may be made."""

    def __init__(self, gates, payload):
        RuntimeError.__init__(self, "quality gates failed: %s" % ", ".join(gates))
        self.gates = list(gates)
        self.payload = payload


# --------------------------------------------------------------------------------------------
# Preregistered index sets (prereg section 5, Q4/Q5)
#
# Built here from the PREREGISTRATION TEXT, independently of
# ppo_update_safety.mirror_navrl_structured_observation.  Q4 then proves the function agrees.
# --------------------------------------------------------------------------------------------


def structured_obs_dim(hbeams, vbeams, max_obstacles):
    """Structured actor observation width implied by the checkpoint's schema knobs."""
    return (
        int(vbeams) * int(hbeams)
        + OBSTACLE_HISTORY * int(max_obstacles) * OBSTACLE_DIM
        + ROBOT_HISTORY * ROBOT_DIM
        + TARGET_HISTORY * TARGET_DIM
    )


def preregistered_scan_permutation(hbeams):
    """``h -> (-h) mod H`` over one LiDAR ring (prereg section 5, Q5)."""
    return dict((h, (-h) % int(hbeams)) for h in range(int(hbeams)))


def preregistered_signed_permutation(hbeams, vbeams, max_obstacles):
    """The full preregistered mirror operator as an explicit signed permutation.

    Returns ``(source, sign)`` such that the preregistered reflection of ``x`` is

        mirrored[:, j] = sign[j] * x[:, source[j]]

    - permutation: ``[0 : vbeams*hbeams]``, per ring ``v`` the index ``v*hbeams + h`` takes its
      value from ``v*hbeams + ((-h) mod hbeams)``;
    - sign flip, obstacle block ``(hist, slot, dim)``: fields 1 and 4;
    - sign flip, robot block ``(hist, dim)``: fields 1, 3, 5, 7;
    - sign flip, target block ``(hist, dim)``: fields 1 and 4;
    - every other index: unchanged.
    """
    hbeams = int(hbeams)
    vbeams = int(vbeams)
    max_obstacles = int(max_obstacles)
    dim = structured_obs_dim(hbeams, vbeams, max_obstacles)
    source = list(range(dim))
    sign = [1] * dim

    ring_map = preregistered_scan_permutation(hbeams)
    for v in range(vbeams):
        for h in range(hbeams):
            source[v * hbeams + h] = v * hbeams + ring_map[h]

    offset = vbeams * hbeams
    for hist in range(OBSTACLE_HISTORY):
        for slot in range(max_obstacles):
            base = offset + (hist * max_obstacles + slot) * OBSTACLE_DIM
            for field in OBSTACLE_SIGN_FLIP_FIELDS:
                sign[base + field] = -1
    offset += OBSTACLE_HISTORY * max_obstacles * OBSTACLE_DIM

    for hist in range(ROBOT_HISTORY):
        base = offset + hist * ROBOT_DIM
        for field in ROBOT_SIGN_FLIP_FIELDS:
            sign[base + field] = -1
    offset += ROBOT_HISTORY * ROBOT_DIM

    for hist in range(TARGET_HISTORY):
        base = offset + hist * TARGET_DIM
        for field in TARGET_SIGN_FLIP_FIELDS:
            sign[base + field] = -1
    offset += TARGET_HISTORY * TARGET_DIM

    if offset != dim:
        raise AssertionError("preregistered block layout does not cover the observation")
    return source, sign


def preregistered_sign_flip_indices(hbeams, vbeams, max_obstacles):
    _source, sign = preregistered_signed_permutation(hbeams, vbeams, max_obstacles)
    return set(index for index, value in enumerate(sign) if value == -1)


def preregistered_permutation_pairs(hbeams, vbeams, max_obstacles):
    """``{destination: source}`` for every index the mirror actually moves (source != dest)."""
    source, _sign = preregistered_signed_permutation(hbeams, vbeams, max_obstacles)
    return dict(
        (dest, src) for dest, src in enumerate(source) if src != dest
    )


# --------------------------------------------------------------------------------------------
# Statistics helpers (prereg section 6)
# --------------------------------------------------------------------------------------------


def percentile(values, q):
    """Deterministic percentile with the documented convention (numpy linear interpolation)."""
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return None
    return float(np.percentile(array, float(q), method=PERCENTILE_METHOD))


def describe(values):
    """median, p90, p95, p99, mean, max, n -- exactly the reported set (prereg section 6)."""
    array = np.asarray(values, dtype=np.float64)
    result = {"n": int(array.size)}
    if array.size == 0:
        result.update(
            {"median": None, "p90": None, "p95": None, "p99": None, "mean": None, "max": None}
        )
        return result
    result["median"] = percentile(array, 50.0)
    for q in PERCENTILE_QUANTILES:
        result["p%d" % int(q)] = percentile(array, q)
    result["mean"] = float(array.mean())
    result["max"] = float(array.max())
    return result


def classify_verdict(median_conj_err_lat, sign_agreement):
    """Prereg section 7 verdict rule.  Thresholds come from module constants only."""
    if median_conj_err_lat is None or sign_agreement is None:
        return None
    if (
        median_conj_err_lat >= VERDICT_CONFIRM_MEDIAN_MIN
        and sign_agreement <= VERDICT_CONFIRM_SIGN_AGREEMENT_MAX
    ):
        return VERDICT_CHIRALITY_CONFIRMED
    if (
        median_conj_err_lat <= VERDICT_ABSENT_MEDIAN_MAX
        and sign_agreement >= VERDICT_ABSENT_SIGN_AGREEMENT_MIN
    ):
        return VERDICT_CHIRALITY_ABSENT
    return VERDICT_INCONCLUSIVE


def has_sufficient_sample(comparable_rows):
    """Prereg section 6: a cell receives a verdict only at >= 256 comparable rows."""
    return int(comparable_rows) >= MIN_CONTEXT_COMPARABLE_ROWS


# --------------------------------------------------------------------------------------------
# Filesystem / provenance helpers
# --------------------------------------------------------------------------------------------


def sha256_file(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_checkpoint(path_argument):
    """Resolve the pinned checkpoint, following the shared git dir when runs/ is not local.

    ``runs/`` is gitignored and exists only in the primary worktree; identity stays pinned by the
    SHA-256 the caller passes.  Same resolution strategy as tools/run_navrl_ref5in_mode_probe.py.
    """
    candidate = Path(path_argument)
    if candidate.is_file():
        return candidate.resolve()
    if candidate.is_absolute():
        raise AuditPreconditionError("checkpoint not found: %s" % candidate)
    local = (ROOT / candidate).resolve()
    if local.is_file():
        return local
    common = Path(
        subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "--git-common-dir"], universal_newlines=True
        ).strip()
    ).resolve()
    primary = (common.parent / candidate).resolve()
    if primary.is_file():
        return primary
    raise AuditPreconditionError("checkpoint not found in worktree or primary: %s" % candidate)


def install_cpu_only_aerial_gym_packages():
    """Make the CPU-only subset of ``aerial_gym`` importable WITHOUT Isaac Gym.

    ``aerial_gym/__init__.py`` does ``import isaacgym`` and then pulls in every task/env/control
    module.  This audit needs only three leaf modules (navrl_perception, navrl_transformer_network,
    navrl_action_models / navrl_players), all of which are simulator-free.  Standing in bare
    packages with the same ``__path__`` -- the pattern tests/test_navrl_ref5in_platform.py already
    uses -- keeps the real module files (and therefore the real builders) while guaranteeing that
    no simulator can be constructed.  ``aerial_gym.rl_training`` and its children are implicit
    namespace packages, so they resolve from the stand-in's ``__path__``.
    """
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    for name, relative in (
        ("aerial_gym", "aerial_gym"),
        ("aerial_gym.task", "aerial_gym/task"),
        ("aerial_gym.task.navrl_task", "aerial_gym/task/navrl_task"),
    ):
        module = sys.modules.get(name)
        if module is None:
            module = types.ModuleType(name)
            sys.modules[name] = module
        if not hasattr(module, "__path__"):
            module.__path__ = [str(ROOT / relative)]
    if not hasattr(sys.modules["aerial_gym"], "AERIAL_GYM_DIRECTORY"):
        sys.modules["aerial_gym"].AERIAL_GYM_DIRECTORY = str(ROOT)


def read_checkpoint_schema(checkpoint_state):
    """Prereg section 5 (Q3): schema knobs come from the CHECKPOINT, never from the environment."""
    env_state = checkpoint_state.get("env_state")
    if not isinstance(env_state, dict):
        raise AuditPreconditionError("checkpoint env_state is missing; cannot read the schema")
    missing = [key for key in CHECKPOINT_ENV_KEYS if env_state.get(key) is None]
    if missing:
        raise AuditPreconditionError(
            "checkpoint lacks schema provenance field(s): %s" % ", ".join(missing)
        )
    schema = {
        "cfg_lidar_hbeams": int(float(env_state["cfg_lidar_hbeams"])),
        "cfg_lidar_vbeams": int(float(env_state["cfg_lidar_vbeams"])),
        "cfg_max_obstacles": int(float(env_state["cfg_max_obstacles"])),
        "cfg_corridor_tokens": int(float(env_state["cfg_corridor_tokens"])),
    }
    if schema["cfg_corridor_tokens"] != 0:
        # mirror_navrl_structured_observation has no corridor-token branch; a nonzero value would
        # silently mirror only a prefix of the observation.
        raise AuditPreconditionError(
            "checkpoint uses %d corridor tokens; the canonical mirror does not support them"
            % schema["cfg_corridor_tokens"]
        )
    schema["structured_obs_dim"] = structured_obs_dim(
        schema["cfg_lidar_hbeams"], schema["cfg_lidar_vbeams"], schema["cfg_max_obstacles"]
    )
    schema["cfg_action_policy"] = str(env_state.get("cfg_action_policy", "") or "").strip()
    schema["cfg_action_std"] = str(env_state.get("cfg_action_std", "") or "").strip()
    schema["cfg_action_mu_scale"] = str(env_state.get("cfg_action_mu_scale", "") or "").strip()
    schema["cfg_truncated_dmin"] = env_state.get("cfg_truncated_dmin")
    schema["cfg_robot_name"] = str(env_state.get("cfg_robot_name", "") or "").strip()
    schema["epoch"] = int(checkpoint_state.get("epoch", -1))
    return schema


def apply_schema_environment(schema):
    """Export the checkpoint's schema so navrl_perception and the mirror agree on ONE source.

    ``mirror_navrl_structured_observation`` reads NAVRL_LIDAR_HBEAMS / NAVRL_LIDAR_VBEAMS /
    NAVRL_MAX_OBSTACLES with defaults 36/4/5 (a 574-D schema) and RAISES on a real 898-D
    observation.  navrl_perception.py freezes the same knobs into module constants AT IMPORT TIME.
    Both are therefore pinned from the checkpoint here, before any of those modules is imported.
    """
    exported = {
        "NAVRL_LIDAR_HBEAMS": str(schema["cfg_lidar_hbeams"]),
        "NAVRL_LIDAR_VBEAMS": str(schema["cfg_lidar_vbeams"]),
        "NAVRL_MAX_OBSTACLES": str(schema["cfg_max_obstacles"]),
        "NAVRL_CORRIDOR_TOKENS": str(schema["cfg_corridor_tokens"]),
    }
    # Action-distribution contract: runner._apply_action_policy_config restores exactly these from
    # the checkpoint for --play.  cfg_action_mu_scale in particular changes the deterministic
    # action (tanh(mu_scale * raw_mu)), so omitting it would evaluate a different controller.
    if schema["cfg_action_std"]:
        exported["NAVRL_ACTION_STD"] = schema["cfg_action_std"]
    if schema["cfg_action_mu_scale"]:
        exported["NAVRL_ACTION_MU_SCALE"] = schema["cfg_action_mu_scale"]
    if schema["cfg_truncated_dmin"] is not None:
        exported["NAVRL_TRUNCATED_DMIN"] = str(schema["cfg_truncated_dmin"])
    if schema["cfg_action_policy"]:
        exported["NAVRL_ACTION_POLICY"] = schema["cfg_action_policy"]
    for key, value in exported.items():
        os.environ[key] = value
    return exported


_ACTION_POLICY_MODELS = {
    "legacy": "continuous_a2c_logstd",
    "fixed_gaussian": "navrl_fixed_gaussian",
    "squashed_gaussian": "navrl_squashed_gaussian",
    "truncated_gaussian": "navrl_truncated_gaussian",
}


def build_player(checkpoint_path, schema, device_name, train_config_path):
    """Instantiate the REAL NavRLPpoPlayerContinuous and restore it -- no environment involved.

    ``BasePlayer.__init__`` builds a vecenv only when ``config['env_info']`` is absent, so the
    env_info that runner.AERIALRLGPUEnv.get_env_info() would have produced is injected directly.
    ``.run()`` is never called.
    """
    import yaml
    import torch
    from gym import spaces
    from rl_games.algos_torch import model_builder

    from aerial_gym.rl_training.rl_games.navrl_transformer_network import NavRLTransformerBuilder
    from aerial_gym.rl_training.rl_games.navrl_action_models import (
        NavRLFixedGaussianModel,
        NavRLSquashedGaussianModel,
        NavRLTruncatedGaussianModel,
    )
    from aerial_gym.rl_training.rl_games.navrl_players import NavRLPpoPlayerContinuous

    # Same performance/precision switches rl_games.torch_runner.Runner.__init__ applies on the
    # live --play path.  They change float32 matmul precision, so replicating them is part of
    # numerical fidelity, not an optimisation.
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.backends.cudnn.benchmark = True
    torch.set_float32_matmul_precision("high")

    model_builder.register_network("navrl_transformer", NavRLTransformerBuilder)
    model_builder.register_model("navrl_fixed_gaussian", NavRLFixedGaussianModel)
    model_builder.register_model("navrl_squashed_gaussian", NavRLSquashedGaussianModel)
    model_builder.register_model("navrl_truncated_gaussian", NavRLTruncatedGaussianModel)

    with open(str(train_config_path), "r") as stream:
        config = yaml.safe_load(stream)
    params = config["params"]

    policy = schema["cfg_action_policy"] or "legacy"
    if policy not in _ACTION_POLICY_MODELS:
        raise AuditPreconditionError("unsupported checkpoint action policy: %r" % policy)
    params["model"]["name"] = _ACTION_POLICY_MODELS[policy]

    obs_dim = int(schema["structured_obs_dim"])
    train_cfg = params["config"]
    train_cfg["env_info"] = {
        "action_space": spaces.Box(-np.ones(4), np.ones(4)),
        "observation_space": spaces.Box(np.ones(obs_dim) * -np.Inf, np.ones(obs_dim) * np.Inf),
        "agents": 1,
        "value_size": 1,
    }
    train_cfg["vec_env"] = None
    train_cfg["device_name"] = device_name
    player_cfg = dict(train_cfg.get("player") or {})
    player_cfg["use_vecenv"] = False
    player_cfg["deterministic"] = True
    player_cfg["print_stats"] = False
    train_cfg["player"] = player_cfg

    player = NavRLPpoPlayerContinuous(params)
    player.restore(str(checkpoint_path))
    player.model.eval()
    if player.model.training:
        raise AuditPreconditionError("restored model is not in eval mode")
    if player.env is not None:
        raise AuditPreconditionError("player unexpectedly constructed an environment")
    return player


# --------------------------------------------------------------------------------------------
# Forward pass -- numerically identical to navrl_players.get_action's diagnostic path
# --------------------------------------------------------------------------------------------


def _running_mean_std(model):
    owner = model
    while hasattr(owner, "_orig_mod"):
        owner = owner._orig_mod
    return owner.running_mean_std


def deterministic_actions(player, obs_batch):
    """One forward pass, returning the same tensor navrl_players would select.

    ``_preproc_obs`` and ``_model_action`` are the player's own methods; ``running_mean_std`` is
    applied inside ``model.forward`` via ``norm_obs``, exactly as in the live path.
    """
    import torch

    from aerial_gym.rl_training.rl_games.navrl_players import NavRLPpoPlayerContinuous

    input_dict = {
        "is_train": False,
        "prev_actions": None,
        "obs": player._preproc_obs(obs_batch),
        "rnn_states": None,
    }
    with torch.no_grad():
        result = player.model(input_dict)
    return NavRLPpoPlayerContinuous._model_action(result, True)


def forward_pairs(player, obs, batch_size, mirror_fn):
    """Deterministic actions for every observation and for its exact reflection.

    MIRROR FIRST, THEN NORMALISE: the raw observation is reflected here and the model's
    running_mean_std is applied afterwards inside the forward, matching navrl_players.py:155.
    """
    import torch

    original_chunks = []
    mirrored_chunks = []
    total = int(obs.shape[0])
    for start in range(0, total, int(batch_size)):
        chunk = obs[start : start + int(batch_size)].to(player.device)
        original_chunks.append(deterministic_actions(player, chunk).detach().cpu())
        mirrored_chunks.append(deterministic_actions(player, mirror_fn(chunk)).detach().cpu())
    if not original_chunks:
        empty = torch.zeros((0, 4), dtype=torch.float32)
        return empty, empty
    return torch.cat(original_chunks, dim=0), torch.cat(mirrored_chunks, dim=0)


# --------------------------------------------------------------------------------------------
# S1 -- exploratory, NON-GATING normaliser-symmetry decomposition (prereg section 6)
# --------------------------------------------------------------------------------------------


def symmetrise_normaliser(running_mean, running_var, source, sign):
    """Symmetrise running_mean_std so that the normaliser COMMUTES with the mirror.

    Write the mirror as a signed permutation ``(M x)_j = s_j * x_{p(j)}`` with ``p`` an involution
    and ``s_{p(j)} = s_j``.  The normaliser is ``N(x)_j = clamp((x_j - m_j) / sqrt(v_j + eps))``.
    Requiring ``N(M x) = M N(x)`` for all ``x`` gives, term by term,

        (s_j x_{p(j)} - m_j) / sqrt(v_j + eps)  ==  s_j * (x_{p(j)} - m_{p(j)}) / sqrt(v_{p(j)}+eps)

    (the clamp is an ODD function on a symmetric interval, so it commutes with the sign and
    imposes no extra condition), which holds for every ``x`` if and only if

        v_j = v_{p(j)}          and          m_j = s_j * m_{p(j)}.

    Hence:
      * VARIANCE is symmetrised by the plain pairwise average, ``v'_j = v'_{p(j)} = (v_j+v_{p(j)})/2``
        -- variance is sign-blind, so a sign-flipped field keeps its own variance and a permuted
        pair shares the average.
      * MEAN is symmetrised as ``m'_j = (m_j + s_j * m_{p(j)}) / 2``.  For an unsigned permuted
        pair (``s=+1``) this is the ordinary average.  For a sign-flipped field the mirror maps the
        index to ITSELF with ``s=-1`` (``p(j)=j``), so the formula collapses to
        ``m'_j = (m_j - m_j)/2 = 0``: the ONLY value a mean can take on a coordinate that must
        change sign under a symmetry of the distribution is zero.  Averaging the raw means there
        instead (``(m_j+m_j)/2 = m_j``) would keep the asymmetry it is supposed to remove.
        The general antisymmetric case ``p(j) != j, s=-1`` is handled by the same expression,
        ``m'_j = (m_j - m_{p(j)})/2 = -m'_{p(j)}``; it does not occur in this schema but costs
        nothing to support.

    Everything is done in float64, the dtype rl_games stores these buffers in.
    """
    import torch

    mean = running_mean.detach().clone().to(torch.float64)
    var = running_var.detach().clone().to(torch.float64)
    index = torch.as_tensor(source, dtype=torch.long, device=mean.device)
    signs = torch.as_tensor(sign, dtype=torch.float64, device=mean.device)
    sym_mean = 0.5 * (mean + signs * mean.index_select(0, index))
    sym_var = 0.5 * (var + var.index_select(0, index))
    return sym_mean, sym_var


# --------------------------------------------------------------------------------------------
# Frame table
# --------------------------------------------------------------------------------------------

# Intended NAVRL_OBS_DUMP schema.  A short, disjoint alias list absorbs harmless naming drift in
# the dump hook; the binding actually used is recorded in the output JSON.
FRAME_KEYS = {
    "obs": ("obs", "observations", "actor_obs"),
    "env_id": ("env_id", "env_ids"),
    "call_index": ("call_index",),
    "episode_uid": ("episode_uid", "frame_episode_uid"),
    "ep_step": ("ep_step", "episode_step"),
    "ctx_target_visible": ("ctx_target_visible", "target_visible"),
    "ctx_front_blocked": ("ctx_front_blocked", "front_blocked"),
    "ctx_valid": ("ctx_valid", "valid_y"),
}
EPISODE_KEYS = {
    "ep_uid": ("ep_uid", "episode_table_uid"),
    "ep_env_id": ("ep_env_id",),
    "outcome": ("outcome", "ep_outcome"),
    "ep_len": ("ep_len", "episode_length"),
}
REQUIRED_FRAME_KEYS = ("obs", "episode_uid", "ctx_target_visible", "ctx_front_blocked", "ctx_valid")
REQUIRED_EPISODE_KEYS = ("ep_uid", "outcome")


def _bind_keys(archive, table, required):
    available = list(archive.keys())
    binding = {}
    for canonical, aliases in sorted(table.items()):
        for alias in aliases:
            if alias in archive:
                binding[canonical] = alias
                break
    missing = [key for key in required if key not in binding]
    if missing:
        raise AuditPreconditionError(
            "frames npz is missing required key(s) %s; keys present: %s"
            % (", ".join(missing), ", ".join(sorted(available)))
        )
    return binding


def load_frames(frames_path):
    archive = np.load(str(frames_path), allow_pickle=False)
    frame_binding = _bind_keys(archive, FRAME_KEYS, REQUIRED_FRAME_KEYS)
    episode_binding = _bind_keys(archive, EPISODE_KEYS, REQUIRED_EPISODE_KEYS)

    frames = dict((key, np.asarray(archive[alias])) for key, alias in frame_binding.items())
    episodes = dict((key, np.asarray(archive[alias])) for key, alias in episode_binding.items())

    obs = frames["obs"]
    if obs.ndim != 2:
        raise AuditPreconditionError("frames obs must be 2-D [N, D], got shape %s" % (obs.shape,))
    n_frames = int(obs.shape[0])
    for key, value in frames.items():
        if key == "obs":
            continue
        if int(value.shape[0]) != n_frames:
            raise AuditPreconditionError(
                "frame column %r has %d rows but obs has %d" % (key, value.shape[0], n_frames)
            )
    n_episodes = int(episodes["ep_uid"].shape[0])
    for key, value in episodes.items():
        if int(value.shape[0]) != n_episodes:
            raise AuditPreconditionError(
                "episode column %r has %d rows but ep_uid has %d"
                % (key, value.shape[0], n_episodes)
            )
    # Provenance scalars the NAVRL_OBS_DUMP writer appends alongside the two tables.  Optional:
    # a dump produced before they existed still audits, it just records nulls.
    extras = {}
    for key in ("stride_eff", "decimations"):
        if key in archive:
            extras[key] = int(np.asarray(archive[key]).reshape(-1)[0])
        else:
            extras[key] = None
    return (
        frames,
        episodes,
        {"frames": frame_binding, "episodes": episode_binding, "dump_provenance": extras},
    )


def join_outcomes(frame_episode_uid, episodes):
    """Map each frame to its episode outcome code; -1 marks a frame with no table entry."""
    lookup = {}
    for uid, code in zip(
        np.asarray(episodes["ep_uid"]).tolist(), np.asarray(episodes["outcome"]).tolist()
    ):
        lookup[int(uid)] = int(code)
    return np.array(
        [lookup.get(int(uid), -1) for uid in np.asarray(frame_episode_uid).tolist()],
        dtype=np.int64,
    )


def build_contexts(frames, outcome_by_frame):
    """Prereg section 6 -- first-order context splits only; never crossed."""
    visible = np.asarray(frames["ctx_target_visible"]).astype(bool)
    front = np.asarray(frames["ctx_front_blocked"]).astype(np.int64)
    contexts = [("overall", np.ones(visible.shape[0], dtype=bool))]
    contexts.append(("target_visible", visible))
    contexts.append(("target_hidden", ~visible))
    contexts.append(("front_blocked", front == 1))
    contexts.append(("front_clear", front == 0))
    contexts.append(("front_unknown", front == -1))
    for code in OUTCOME_CONTEXT_CODES:
        contexts.append(("outcome_" + OUTCOME_CODES[code], outcome_by_frame == code))
    return contexts


# --------------------------------------------------------------------------------------------
# Measurements
# --------------------------------------------------------------------------------------------


def measure_cell(original, mirrored, mask):
    """All preregistered statistics for one context cell (numpy float64 throughout)."""
    rows = np.asarray(mask, dtype=bool)
    pi_o = np.asarray(original, dtype=np.float64)[rows]
    pi_mo = np.asarray(mirrored, dtype=np.float64)[rows]
    n_rows = int(pi_o.shape[0])

    # e = pi(preproc(M o)) - mirror_navrl_actions(pi(preproc(o))); the action mirror negates
    # channels 1 (lateral) and 3 (yaw).
    expected = pi_o.copy()
    expected[:, 1] = -expected[:, 1]
    expected[:, 3] = -expected[:, 3]
    error = pi_mo - expected

    cell = {"n_rows": n_rows, "channels": {}}
    for name, axis in ACTION_CHANNELS:
        cell["channels"][name] = describe(np.abs(error[:, axis]))

    lat_o = pi_o[:, LATERAL_ACTION_INDEX]
    lat_mo = pi_mo[:, LATERAL_ACTION_INDEX]
    comparable = (np.abs(lat_o) >= LATERAL_SIGN_THRESHOLD) & (
        np.abs(lat_mo) >= LATERAL_SIGN_THRESHOLD
    )
    n_comparable = int(comparable.sum())
    if n_comparable > 0:
        agreeing = np.sign(lat_mo[comparable]) == -np.sign(lat_o[comparable])
        sign_agreement = float(agreeing.sum()) / float(n_comparable)
    else:
        sign_agreement = None
    cell["lateral_sign_agreement"] = sign_agreement
    cell["lateral_sign_comparable"] = n_comparable
    cell["lateral_sign_threshold"] = LATERAL_SIGN_THRESHOLD
    cell["signed_lateral_bias"] = (
        float((lat_o.mean() + lat_mo.mean()) / 2.0) if n_rows > 0 else None
    )
    cell["mean_pi_o_lateral"] = float(lat_o.mean()) if n_rows > 0 else None
    cell["mean_pi_mo_lateral"] = float(lat_mo.mean()) if n_rows > 0 else None

    sufficient = has_sufficient_sample(n_comparable)
    cell["insufficient_sample"] = not sufficient
    cell["min_comparable_rows"] = MIN_CONTEXT_COMPARABLE_ROWS
    if sufficient:
        cell["verdict"] = classify_verdict(
            cell["channels"]["conj_err_lat"]["median"], sign_agreement
        )
    else:
        cell["verdict"] = None
    return cell


def measure_all(original, mirrored, contexts):
    return dict(
        (name, measure_cell(original, mirrored, mask)) for name, mask in contexts
    )


# --------------------------------------------------------------------------------------------
# Quality gates (prereg section 5)
# --------------------------------------------------------------------------------------------


def gate_schema(schema, obs_width):
    """Q3 -- schema from checkpoint metadata, and it must match the collected observation."""
    derived = int(schema["structured_obs_dim"])
    detail = {
        "cfg_lidar_hbeams": schema["cfg_lidar_hbeams"],
        "cfg_lidar_vbeams": schema["cfg_lidar_vbeams"],
        "cfg_max_obstacles": schema["cfg_max_obstacles"],
        "cfg_corridor_tokens": schema["cfg_corridor_tokens"],
        "derived_structured_obs_dim": derived,
        "frames_obs_dim": int(obs_width),
        "expected": GATE_STRUCTURED_OBS_DIM,
    }
    passed = derived == int(obs_width) == GATE_STRUCTURED_OBS_DIM
    return passed, detail


def gate_index_sets(mirror_fn, hbeams, vbeams, max_obstacles):
    """Q4 -- byte-level index-set exactness against the independently built preregistered sets."""
    import torch

    source, sign = preregistered_signed_permutation(hbeams, vbeams, max_obstacles)
    dim = len(source)

    # 1. Recover the operator the function actually implements.  Distinct positive magnitudes make
    #    (source index, sign) unambiguous for every output coordinate.
    probe = torch.arange(1, dim + 1, dtype=torch.float32).view(1, dim)
    probed = mirror_fn(probe)[0]
    observed_source = (probed.abs() - 1.0).round().to(torch.long).tolist()
    observed_sign = [1 if float(value) > 0.0 else -1 for value in probed.tolist()]

    observed_flip = set(i for i, s in enumerate(observed_sign) if s == -1)
    observed_perm = dict((d, s) for d, s in enumerate(observed_source) if s != d)
    expected_flip = set(i for i, s in enumerate(sign) if s == -1)
    expected_perm = dict((d, s) for d, s in enumerate(source) if s != d)
    observed_unchanged = set(
        i
        for i in range(dim)
        if observed_source[i] == i and observed_sign[i] == 1
    )
    expected_unchanged = set(range(dim)) - expected_flip - set(expected_perm)

    # 2. Byte-level equality on a synthetic random tensor against an independently built expected
    #    output (this is what makes "every other index is unchanged" a real assertion).
    generator = torch.Generator().manual_seed(20260821)
    random_obs = torch.randn((8, dim), generator=generator, dtype=torch.float32)
    index = torch.as_tensor(source, dtype=torch.long)
    signs = torch.as_tensor(sign, dtype=torch.float32)
    expected_obs = random_obs.index_select(1, index) * signs
    byte_exact = bool(torch.equal(mirror_fn(random_obs), expected_obs))

    passed = (
        observed_flip == expected_flip
        and observed_perm == expected_perm
        and observed_unchanged == expected_unchanged
        and byte_exact
    )
    detail = {
        "sign_flip_index_count": len(expected_flip),
        "sign_flip_index_sets_match": observed_flip == expected_flip,
        "permutation_index_count": len(expected_perm),
        "permutation_index_sets_match": observed_perm == expected_perm,
        "unchanged_index_count": len(expected_unchanged),
        "unchanged_index_sets_match": observed_unchanged == expected_unchanged,
        "byte_level_equality_on_random_tensor": byte_exact,
        "probe_rows": int(random_obs.shape[0]),
    }
    return passed, detail, observed_source, observed_sign


def gate_scan_permutation(observed_source, hbeams, vbeams):
    """Q5 -- ring permutation h -> (-h) mod H with fixed points exactly {0, 36}."""
    hbeams = int(hbeams)
    vbeams = int(vbeams)
    expected = preregistered_scan_permutation(hbeams)
    rings_match = True
    for v in range(vbeams):
        for h in range(hbeams):
            if observed_source[v * hbeams + h] != v * hbeams + expected[h]:
                rings_match = False
                break
    fixed_points = set(h for h in range(hbeams) if expected[h] == h)
    detail = {
        "hbeams": hbeams,
        "vbeams": vbeams,
        "ring_permutation_matches_all_rings": rings_match,
        "fixed_points": sorted(fixed_points),
        "expected_fixed_points": sorted(GATE_SCAN_FIXED_POINTS),
    }
    passed = rings_match and fixed_points == set(GATE_SCAN_FIXED_POINTS)
    return passed, detail


def gate_involution_and_isometry(obs, mirror_fn, batch_size, device):
    """Q1 -- M(M(x)) == x exactly; Q2 -- ||M x|| == ||x|| to 1e-3.  Over every collected frame."""
    import torch

    max_involution = 0.0
    max_isometry = 0.0
    total = int(obs.shape[0])
    for start in range(0, total, int(batch_size)):
        chunk = obs[start : start + int(batch_size)].to(device)
        mirrored = mirror_fn(chunk)
        round_trip = mirror_fn(mirrored)
        max_involution = max(
            max_involution, float((round_trip - chunk).abs().max().item()) if chunk.numel() else 0.0
        )
        norm_delta = torch.linalg.norm(mirrored, dim=1) - torch.linalg.norm(chunk, dim=1)
        max_isometry = max(
            max_isometry, float(norm_delta.abs().max().item()) if chunk.numel() else 0.0
        )
    involution_passed = max_involution == GATE_INVOLUTION_MAX_ABS
    isometry_passed = max_isometry <= GATE_ISOMETRY_MAX_ABS
    return (
        involution_passed,
        {"max_abs_round_trip_error": max_involution, "threshold": GATE_INVOLUTION_MAX_ABS},
        isometry_passed,
        {"max_abs_norm_difference": max_isometry, "threshold": GATE_ISOMETRY_MAX_ABS},
    )


# --------------------------------------------------------------------------------------------
# Equivalence proof (self-check)
# --------------------------------------------------------------------------------------------


def equivalence_proof(player, checkpoint_state, device):
    """Prove the offline forward reproduces the live path, with reportable numbers."""
    import torch

    stripped = {}
    for key, value in checkpoint_state["model"].items():
        stripped[key[len("_orig_mod.") :] if key.startswith("_orig_mod.") else key] = value

    model_state = player.model.state_dict()
    keys_match = sorted(model_state.keys()) == sorted(stripped.keys())

    def _fingerprint(state):
        digest = hashlib.sha256()
        count = 0
        for key in sorted(state.keys()):
            tensor = state[key].detach().to("cpu").contiguous()
            digest.update(key.encode("utf-8"))
            digest.update(str(tuple(tensor.shape)).encode("utf-8"))
            digest.update(str(tensor.dtype).encode("utf-8"))
            digest.update(tensor.reshape(-1).numpy().tobytes())
            count += int(tensor.numel())
        return digest.hexdigest(), count

    model_hash, model_elements = _fingerprint(model_state)
    checkpoint_hash, checkpoint_elements = _fingerprint(stripped)

    rms = _running_mean_std(player.model)
    rms_mean_identical = bool(
        torch.equal(rms.running_mean.detach().cpu(), stripped["running_mean_std.running_mean"].cpu())
    )
    rms_var_identical = bool(
        torch.equal(rms.running_var.detach().cpu(), stripped["running_mean_std.running_var"].cpu())
    )
    rms_count_identical = bool(
        torch.equal(rms.count.detach().cpu(), stripped["running_mean_std.count"].cpu())
    )

    from aerial_gym.rl_training.rl_games.navrl_players import NavRLPpoPlayerContinuous

    dim = int(rms.running_mean.numel())
    generator = torch.Generator().manual_seed(1900)
    synthetic = torch.randn((16, dim), generator=generator, dtype=torch.float32).to(device)
    with torch.no_grad():
        res_dict = player.model(
            {
                "is_train": False,
                "prev_actions": None,
                "obs": player._preproc_obs(synthetic),
                "rnn_states": None,
            }
        )
    has_deterministic = "deterministic_actions" in res_dict
    selected = NavRLPpoPlayerContinuous._model_action(res_dict, True)
    picks_deterministic = has_deterministic and bool(
        torch.equal(selected, res_dict["deterministic_actions"])
    )
    tanh_gap = float((selected - torch.tanh(res_dict["mus"])).abs().max().item())
    mus_gap = float((selected - res_dict["mus"]).abs().max().item())

    return {
        "state_dict_keys_match": keys_match,
        "model_parameter_element_count": model_elements,
        "checkpoint_parameter_element_count": checkpoint_elements,
        "model_parameter_sha256": model_hash,
        "checkpoint_parameter_sha256": checkpoint_hash,
        "parameter_fingerprints_identical": model_hash == checkpoint_hash,
        "parameter_fingerprint_recipe": (
            "sha256 over sorted state_dict keys of (key, shape, dtype, raw little-endian bytes of "
            "the flattened tensor)"
        ),
        "running_mean_bitwise_identical_to_checkpoint": rms_mean_identical,
        "running_var_bitwise_identical_to_checkpoint": rms_var_identical,
        "running_count_bitwise_identical_to_checkpoint": rms_count_identical,
        "running_mean_std_source": "checkpoint model state_dict (never recomputed from frames)",
        "res_dict_keys": sorted(res_dict.keys()),
        "model_action_prefers_deterministic_actions": picks_deterministic,
        "max_abs_selected_minus_tanh_mus": tanh_gap,
        "max_abs_selected_minus_mus": mus_gap,
        "action_units": "normalised policy output before rescale/clamp to actions_low/high",
        "model_in_eval_mode": not player.model.training,
        "player_created_environment": player.env is not None,
    }


# --------------------------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------------------------


def write_json(path, payload):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.name + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(out)


def _base_payload(args, checkpoint_path, checkpoint_sha, frames_sha, schema, exported_env):
    return {
        "schema_version": SCHEMA_VERSION,
        "producer": PRODUCER,
        "created_at_utc": datetime.utcnow().isoformat() + "Z",
        "preregistration": "docs/prereg_2026-08-21_n1_real_frame_reflection_audit.md",
        "scope": "offline_real_frame_reflection_audit_no_simulator",
        "decision_authority": "none",
        "simulator_created": False,
        "environment_stepped": False,
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": checkpoint_sha,
        "checkpoint_epoch": schema["epoch"],
        "robot_name": schema["cfg_robot_name"],
        "frames_npz": str(Path(args.frames).resolve()),
        "frames_sha256": frames_sha,
        "checkpoint_schema": dict(
            (key, schema[key])
            for key in (
                "cfg_lidar_hbeams",
                "cfg_lidar_vbeams",
                "cfg_max_obstacles",
                "cfg_corridor_tokens",
                "structured_obs_dim",
                "cfg_action_policy",
                "cfg_action_std",
                "cfg_action_mu_scale",
            )
        ),
        "schema_environment_exported": exported_env,
        "percentile_convention": PERCENTILE_CONVENTION,
        "thresholds": {
            "verdict_confirm_median_min": VERDICT_CONFIRM_MEDIAN_MIN,
            "verdict_confirm_sign_agreement_max": VERDICT_CONFIRM_SIGN_AGREEMENT_MAX,
            "verdict_absent_median_max": VERDICT_ABSENT_MEDIAN_MAX,
            "verdict_absent_sign_agreement_min": VERDICT_ABSENT_SIGN_AGREEMENT_MIN,
            "lateral_sign_threshold": LATERAL_SIGN_THRESHOLD,
            "min_context_comparable_rows": MIN_CONTEXT_COMPARABLE_ROWS,
            "min_valid_frames": MIN_VALID_FRAMES,
            "gate_involution_max_abs": GATE_INVOLUTION_MAX_ABS,
            "gate_isometry_max_abs": GATE_ISOMETRY_MAX_ABS,
        },
        "outcome_code_map": dict((str(code), name) for code, name in OUTCOME_CODES.items()),
    }


def run_audit(args):
    checkpoint_path = resolve_checkpoint(args.checkpoint)
    checkpoint_sha = sha256_file(checkpoint_path)
    if checkpoint_sha != str(args.checkpoint_sha256).strip().lower():
        raise AuditPreconditionError(
            "checkpoint SHA-256 mismatch: expected %s, got %s"
            % (args.checkpoint_sha256, checkpoint_sha)
        )
    frames_path = Path(args.frames).resolve()
    if not frames_path.is_file():
        raise AuditPreconditionError("frames npz not found: %s" % frames_path)
    frames_sha = sha256_file(frames_path)
    if args.frames_sha256 and frames_sha != str(args.frames_sha256).strip().lower():
        raise AuditPreconditionError(
            "frames SHA-256 mismatch: expected %s, got %s" % (args.frames_sha256, frames_sha)
        )

    import torch  # noqa: F401  (imported here so env vars below are set before aerial_gym loads)

    checkpoint_state = torch.load(str(checkpoint_path), map_location="cpu", weights_only=False)
    schema = read_checkpoint_schema(checkpoint_state)
    exported_env = apply_schema_environment(schema)

    install_cpu_only_aerial_gym_packages()
    from aerial_gym.rl_training.rl_games.ppo_update_safety import (
        mirror_navrl_structured_observation,
    )

    frames, episodes, key_binding = load_frames(frames_path)
    obs_all = torch.as_tensor(np.ascontiguousarray(frames["obs"]), dtype=torch.float32)
    valid = np.asarray(frames["ctx_valid"]).astype(bool)

    payload = _base_payload(args, checkpoint_path, checkpoint_sha, frames_sha, schema, exported_env)
    payload["npz_key_binding"] = key_binding
    payload["frames"] = {
        "n_frames_total": int(obs_all.shape[0]),
        "n_frames_valid": int(valid.sum()),
        "obs_dim": int(obs_all.shape[1]),
        "n_episodes": int(np.asarray(episodes["ep_uid"]).shape[0]),
    }

    device = torch.device(args.device)
    gates = {}

    passed, detail = gate_schema(schema, int(obs_all.shape[1]))
    gates["Q3_schema"] = {"passed": passed, **detail}

    if passed:
        q4_passed, q4_detail, observed_source, observed_sign = gate_index_sets(
            mirror_navrl_structured_observation,
            schema["cfg_lidar_hbeams"],
            schema["cfg_lidar_vbeams"],
            schema["cfg_max_obstacles"],
        )
        gates["Q4_index_sets"] = {"passed": q4_passed, **q4_detail}
        q5_passed, q5_detail = gate_scan_permutation(
            observed_source, schema["cfg_lidar_hbeams"], schema["cfg_lidar_vbeams"]
        )
        gates["Q5_scan_permutation"] = {"passed": q5_passed, **q5_detail}
    else:
        observed_source = observed_sign = None
        gates["Q4_index_sets"] = {"passed": False, "skipped": "Q3 failed"}
        gates["Q5_scan_permutation"] = {"passed": False, "skipped": "Q3 failed"}

    n_valid = int(valid.sum())
    gates["Q9_sample_size"] = {
        "passed": n_valid >= MIN_VALID_FRAMES,
        "n_valid_frames": n_valid,
        "threshold": MIN_VALID_FRAMES,
    }

    if gates["Q3_schema"]["passed"]:
        q1_passed, q1_detail, q2_passed, q2_detail = gate_involution_and_isometry(
            obs_all, mirror_navrl_structured_observation, args.batch_size, device
        )
    else:
        q1_passed, q1_detail = False, {"skipped": "Q3 failed"}
        q2_passed, q2_detail = False, {"skipped": "Q3 failed"}
    gates["Q1_involution"] = {"passed": q1_passed, **q1_detail}
    gates["Q2_isometry"] = {"passed": q2_passed, **q2_detail}

    gates["Q6_import_origin"] = {
        "passed": None,
        "owner": "launcher",
        "note": "import-origin enforcement is the launcher's gate; not evaluated here",
    }
    gates["Q7_checkpoint_sha_and_manifest"] = {
        "passed": True,
        "checkpoint_sha256_matches": True,
        "note": "manifest / schema_version 2 receipt checks are the launcher's gate",
    }

    failed = [name for name, info in sorted(gates.items()) if info.get("passed") is False]
    if failed:
        payload["quality_gates"] = gates
        payload["verdict"] = VERDICT_FAIL_CLOSED
        payload["failed_gates"] = failed
        payload["note"] = (
            "One or more preregistered transform-quality gates failed. No policy statistics and no "
            "policy claim are reported (prereg section 5)."
        )
        raise GateFailure(failed, payload)

    # ---- forward passes -------------------------------------------------------------------
    player = build_player(checkpoint_path, schema, args.device, args.train_config)
    payload["equivalence_proof"] = equivalence_proof(player, checkpoint_state, device)
    payload["forward_contract"] = {
        "order": "mirror in raw units, then normalise inside model.norm_obs (navrl_players.py:155)",
        "normaliser": "running_mean_std restored from the checkpoint; never recomputed",
        "action_selection": "deterministic_actions (post-tanh) via NavRLPpoPlayerContinuous._model_action",
        "grad_mode": "torch.no_grad",
        "model_mode": "eval",
        "device": str(device),
        "batch_size": int(args.batch_size),
        "tf32": {
            "cuda_matmul_allow_tf32": bool(torch.backends.cuda.matmul.allow_tf32),
            "cudnn_allow_tf32": bool(torch.backends.cudnn.allow_tf32),
            "float32_matmul_precision": torch.get_float32_matmul_precision(),
            "note": "replicates rl_games.torch_runner.Runner.__init__ on the live --play path",
        },
    }

    original_a, mirrored_a = forward_pairs(
        player, obs_all, args.batch_size, mirror_navrl_structured_observation
    )
    original_b, mirrored_b = forward_pairs(
        player, obs_all, args.batch_size, mirror_navrl_structured_observation
    )
    q8_passed = bool(torch.equal(original_a, original_b) and torch.equal(mirrored_a, mirrored_b))
    gates["Q8_determinism"] = {
        "passed": q8_passed,
        "original_actions_bitwise_identical": bool(torch.equal(original_a, original_b)),
        "mirrored_actions_bitwise_identical": bool(torch.equal(mirrored_a, mirrored_b)),
        "repeats": 2,
    }
    if not q8_passed:
        payload["quality_gates"] = gates
        payload["verdict"] = VERDICT_FAIL_CLOSED
        payload["failed_gates"] = ["Q8_determinism"]
        raise GateFailure(["Q8_determinism"], payload)

    payload["quality_gates"] = gates

    # ---- measurements ---------------------------------------------------------------------
    outcome_by_frame = join_outcomes(frames["episode_uid"], episodes)
    valid_index = np.nonzero(valid)[0]
    valid_frames = dict(
        (key, np.asarray(value)[valid_index]) for key, value in frames.items() if key != "obs"
    )
    contexts = build_contexts(valid_frames, outcome_by_frame[valid_index])

    original_np = original_a.numpy()[valid_index]
    mirrored_np = mirrored_a.numpy()[valid_index]
    payload["measurements_raw_normaliser"] = measure_all(original_np, mirrored_np, contexts)
    payload["frames"]["n_frames_unattributed_outcome"] = int(
        (outcome_by_frame[valid_index] == -1).sum()
    )
    payload["frames"]["outcome_frame_counts"] = dict(
        (OUTCOME_CODES[code], int((outcome_by_frame[valid_index] == code).sum()))
        for code in sorted(OUTCOME_CODES)
    )

    # ---- S1: exploratory, NON-GATING normaliser-symmetry decomposition ---------------------
    source, sign = preregistered_signed_permutation(
        schema["cfg_lidar_hbeams"], schema["cfg_lidar_vbeams"], schema["cfg_max_obstacles"]
    )
    sym_player = build_player(checkpoint_path, schema, args.device, args.train_config)
    sym_rms = _running_mean_std(sym_player.model)
    sym_mean, sym_var = symmetrise_normaliser(sym_rms.running_mean, sym_rms.running_var, source, sign)
    with torch.no_grad():
        sym_rms.running_mean.copy_(sym_mean)
        sym_rms.running_var.copy_(sym_var)
    sym_original, sym_mirrored = forward_pairs(
        sym_player, obs_all, args.batch_size, mirror_navrl_structured_observation
    )
    payload["s1_symmetrised_normaliser"] = {
        "gating": False,
        "exploratory": True,
        "purpose": (
            "prereg section 6 S1: decompose measured chirality into network vs normaliser "
            "asymmetry. e_sym ~ e_raw means the network carries it; e_sym << e_raw means the "
            "normaliser statistics do. NOT used for any verdict (prereg L2)."
        ),
        "symmetrisation": {
            "variance": "v'_j = v'_{p(j)} = (v_j + v_{p(j)}) / 2 (pairwise average; variance is sign-blind)",
            "mean": "m'_j = (m_j + s_j * m_{p(j)}) / 2",
            "reasoning": (
                "The mirror is a signed permutation (M x)_j = s_j x_{p(j)} with p an involution "
                "and s_{p(j)} = s_j. Requiring the normaliser to COMMUTE with it, N(Mx) = M N(x), "
                "forces v_j = v_{p(j)} and m_j = s_j m_{p(j)}. The clamp to [-5, 5] is odd on a "
                "symmetric interval and imposes no further condition. For a sign-flipped field the "
                "mirror maps the index to ITSELF with s = -1, so the mean formula collapses to "
                "m'_j = (m_j - m_j)/2 = 0: a coordinate that must change sign under the symmetry "
                "can only have mean zero. Plain averaging would have left m_j unchanged there and "
                "removed nothing."
            ),
            "dtype": "float64 (the dtype rl_games stores running_mean_std in)",
            "sign_flipped_indices_forced_to_zero_mean": int(sum(1 for s in sign if s == -1)),
        },
        "measurements": measure_all(
            sym_original.numpy()[valid_index], sym_mirrored.numpy()[valid_index], contexts
        ),
    }

    overall = payload["measurements_raw_normaliser"]["overall"]
    payload["verdict"] = overall["verdict"] if overall["verdict"] else VERDICT_INCONCLUSIVE
    payload["verdict_basis"] = {
        "cell": "overall",
        "median_conj_err_lat": overall["channels"]["conj_err_lat"]["median"],
        "lateral_sign_agreement": overall["lateral_sign_agreement"],
        "comparable_rows": overall["lateral_sign_comparable"],
        "insufficient_sample": overall["insufficient_sample"],
    }
    payload["p2_verdict_changed"] = False
    payload["d1_verdict_changed"] = False
    payload["p3_unlocked"] = False
    return payload


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--frames", required=True, help="NAVRL_OBS_DUMP .npz of real observations")
    parser.add_argument("--checkpoint", required=True, help="frozen policy checkpoint (.pth)")
    parser.add_argument(
        "--checkpoint-sha256", required=True, help="expected checkpoint SHA-256 (verified first)"
    )
    parser.add_argument("--frames-sha256", default="", help="optional expected frames SHA-256")
    parser.add_argument("--out", required=True, help="output summary JSON path")
    parser.add_argument(
        "--device",
        default="auto",
        help="torch device for the forward passes ('auto' = cuda when available)",
    )
    parser.add_argument("--batch-size", type=int, default=DEFAULT_BATCH_SIZE)
    parser.add_argument(
        "--train-config",
        default=str(DEFAULT_TRAIN_CONFIG),
        help="rl_games training YAML the checkpoint was produced with",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if str(args.device).strip().lower() == "auto":
        import torch

        args.device = "cuda" if torch.cuda.is_available() else "cpu"
    try:
        payload = run_audit(args)
    except GateFailure as failure:
        write_json(args.out, failure.payload)
        print(
            "[reflection-audit] %s | failed gates: %s -> %s"
            % (VERDICT_FAIL_CLOSED, ", ".join(failure.gates), args.out),
            file=sys.stderr,
        )
        return 3
    except AuditPreconditionError as exc:
        print("[reflection-audit] PRECONDITION FAILED | %s" % exc, file=sys.stderr)
        return 2
    write_json(args.out, payload)
    print(
        "[reflection-audit] %s | median(conj_err_lat)=%s sign_agreement=%s -> %s"
        % (
            payload["verdict"],
            payload["verdict_basis"]["median_conj_err_lat"],
            payload["verdict_basis"]["lateral_sign_agreement"],
            args.out,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
