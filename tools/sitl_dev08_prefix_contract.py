"""Offline input and outcome contracts for an opt-in dev08 observed-prefix diagnostic.

No simulator, bridge, policy or process is started. Default strict600 remains
blocked; observed-prefix-v1 verifies recorded inputs without promising launch
readiness or a 60-second episode outcome. Historical files are read only.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics
import struct


ROOT = Path(__file__).resolve().parents[1]
INVENTORY = "docs/evidence/sitl_2026-11-05/dev08/input_inventory.json"
INVENTORY_SHA = "379c02fac58b62fdd160ab7c6c6f99065d8c4939044cc97931e5fcac73fcd12d"
CONFIG_SOURCE = "tools/sitl_s3b_dev08.py"
CONFIG_SHA = "e63049cc92bae56afad212219e42af9f91def1f2fac88f688e69a0065a8fa3fb"
F_SHA = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
FRAME = struct.Struct("<HHI20fBbi")
DT = 0.1
CAP = 600  # recorded checkpoints 0..599, not the unrecorded state after action600
ENVS = tuple(range(10))
ARMS = ("native", "candidate")
CENSORED = "CENSORED_TRACE_END"
MAX_FILE_BYTES = 512 * 1024**2
FRAME_CONTRACT = "frame k = state after k completed actions; recorded when its observation is consumed"


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_file(path, record):
    size = record["bytes"]
    if type(size) is not int or not 0 <= size <= MAX_FILE_BYTES:
        raise ValueError("invalid or oversized recorded input")
    if Path(path).stat().st_size != size or sha256_file(path) != record["sha256"]:
        raise ValueError("input size/hash differs: " + str(path))


def selected_frames(stream, count):
    """Decode the pinned v1 wire format; keep only env0..9/episode0."""
    if type(count) is not int or not 0 < count <= MAX_FILE_BYTES // FRAME.size:
        raise ValueError("invalid frame count")
    grouped = {env: {} for env in ENVS}
    for _ in range(count):
        raw = stream.read(FRAME.size)
        if len(raw) != FRAME.size:
            raise ValueError("truncated frame record")
        row = FRAME.unpack(raw)
        env, k, episode = row[:3]
        if env not in grouped or episode != 0:
            continue
        if k >= CAP or k in grouped[env] or not all(math.isfinite(v) for v in row[3:23]):
            raise ValueError("duplicate, nonfinite or out-of-contract selected frame")
        grouped[env][k] = row
    if stream.read(1):
        raise ValueError("extra bytes after declared frame count")
    for env, rows in grouped.items():
        if not rows or sorted(rows) != list(range(len(rows))):
            raise ValueError("missing or noncontiguous env%d episode0" % env)
    return grouped


def frame_window(recorded_frames):
    if type(recorded_frames) is not int or not 1 <= recorded_frames <= CAP:
        raise ValueError("recorded checkpoint count outside 1..600")
    last = recorded_frames - 1
    return {"recorded_frames": recorded_frames, "last_frame_index": last,
            "last_scheduled_elapsed_s": round(last * DT, 10),
            "max_commands_before_last_check": last,
            "unresolved_at_last_check": CENSORED,
            "post_action600_state_available": False}


def checkpoint_decision(k, recorded_frames, capture=False, crash=False):
    """Physical terminal wins before administrative censoring; capture keeps legacy precedence.

    Future runtime checks state first, and emits no command on a terminal/censor
    checkpoint. Never clamp a requested target index or issue an unscored tail action.
    """
    window = frame_window(recorded_frames)
    if type(k) is not int or not 0 <= k <= window["last_frame_index"]:
        raise ValueError("target checkpoint is unavailable")
    if type(capture) is not bool or type(crash) is not bool:
        raise ValueError("terminal predicates must be booleans")
    if capture:
        return "capture"
    if crash:
        return "crash"
    return CENSORED if k == window["last_frame_index"] else "CONTINUE"


def summarize_complete_pairs(records, windows):
    """Validate a complete future main ledger, then give descriptive known/unknown counts.

    No measured result is supplied by this repository unit. Bounds are logical
    counts under arbitrary unobserved continuation, not confidence intervals or
    identified 60-second performance. Harness errors/incomplete cohorts refuse.
    """
    expected = {(bars, env, arm) for bars in (70, 205) for env in ENVS for arm in ARMS}
    expected_windows = {(bars, env) for bars in (70, 205) for env in ENVS}
    if set(windows) != expected_windows:
        raise ValueError("incomplete or extra fixed window identity")
    for key, window in windows.items():
        canonical = frame_window(208 if key == (70, 5) else 600)
        if any(window.get(k) != v for k, v in canonical.items()):
            raise ValueError("window differs from the fixed recorded cohort")
    by = {}
    for record in records:
        if type(record["bars"]) is not int or type(record["env"]) is not int:
            raise ValueError("episode identifiers must be integers")
        if (type(record["seed"]) is not int or record["seed"] != 4501
                or type(record["env_episode_index"]) is not int or record["env_episode_index"] != 0):
            raise ValueError("wrong seed or episode index")
        key = (record["bars"], record["env"], record["arm"])
        if key not in expected or key in by or record.get("phase") != "main":
            raise ValueError("duplicate, extra or non-main episode")
        if record.get("protocol") != "observed-prefix-v1" or record.get("operational_status") != "VALID":
            raise ValueError("wrong protocol or failed/unverified operational record")
        window = windows[(key[0], key[1])]
        if any(window.get(k) != v for k, v in frame_window(window["recorded_frames"]).items()):
            raise ValueError("inconsistent frame window")
        k = record["final_frame_index"]
        if type(k) is not int or not 0 <= k <= window["last_frame_index"]:
            raise ValueError("outcome frame outside recorded prefix")
        if type(record["observation_checks"]) is not int or record["observation_checks"] != k + 1:
            raise ValueError("observation count does not match final checkpoint")
        if type(record["commands_issued"]) is not int or record["commands_issued"] != k:
            raise ValueError("unobserved or post-terminal command")
        times = [record["start_sim_time_s"], record["final_sim_time_s"], record["scheduled_elapsed_s"]]
        if any(type(value) not in (int, float) or not math.isfinite(value) or value < 0 for value in times):
            raise ValueError("missing/nonfinite simulator or schedule time")
        if times[1] < times[0] or times[2] != round(k * DT, 10):
            raise ValueError("nonmonotonic simulator time or wrong nominal checkpoint time")
        outcome = record["outcome"]
        if outcome not in ("capture", "crash", CENSORED):
            raise ValueError("timeout/error is not an observed-prefix outcome")
        if outcome == CENSORED and k != window["last_frame_index"]:
            raise ValueError("premature censor cannot hide a harness failure")
        by[key] = record
    if set(by) != expected:
        raise ValueError("incomplete fixed40 main ledger")
    def counts_for(rows, planned):
        counts = {label: sum(r["outcome"] == label for r in rows)
                  for label in ("capture", "crash", CENSORED)}
        return {"planned": planned, "counts": counts,
                "logical_count_bounds": {label: {"lower": counts[label],
                                                   "upper": counts[label] + counts[CENSORED],
                                                   "denominator": planned}
                                         for label in ("capture", "crash")}}

    summaries = {}
    for arm in ARMS:
        rows = [r for key, r in by.items() if key[2] == arm]
        summaries[arm] = counts_for(rows, 20)
    transitions, differences, unknown = {}, [], 0
    for bars in (70, 205):
        for env in ENVS:
            a, b = (by[(bars, env, arm)] for arm in ARMS)
            key = a["outcome"] + "->" + b["outcome"]
            transitions[key] = transitions.get(key, 0) + 1
            unknown += int(CENSORED in (a["outcome"], b["outcome"]))
            if a["outcome"] == b["outcome"] == "capture":
                differences.append(round((b["final_frame_index"] - a["final_frame_index"]) * DT, 10))
    return {"scope": "observed-prefix paired development diagnostic only",
            "arms": summaries,
            "by_density": {str(bars): {arm: counts_for([r for key, r in by.items()
                                                         if key[0] == bars and key[2] == arm], 10)
                                       for arm in ARMS} for bars in (70, 205)},
            "paired_transitions": transitions, "pairs_with_censoring": unknown,
            "candidate_minus_native_capture_checkpoint_s": {
                "n": len(differences), "median": statistics.median(differences) if differences else None,
                "min": min(differences) if differences else None, "max": max(differences) if differences else None},
            "bounds_are_confidence_intervals": False, "full_60s_outcome_identified": False}


def prepare(root=ROOT, protocol="strict600"):
    if protocol not in ("strict600", "observed-prefix-v1"):
        raise ValueError("unknown protocol")
    root = Path(root)
    if sha256_file(root / INVENTORY) != INVENTORY_SHA or sha256_file(root / CONFIG_SOURCE) != CONFIG_SHA:
        raise ValueError("preparation inventory/configuration source drift")
    inventory = json.loads((root / INVENTORY).read_text())
    spec = importlib.util.spec_from_file_location("_dev08_configuration", root / CONFIG_SOURCE)
    config_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config_module)  # pinned stdlib-only static preparation, no runner import
    configuration = config_module.prepare(root)
    verified = 0
    for cell in inventory["cells"].values():
        for arm in ("full", "zero"):
            for name, record in cell[arm]["files"].items():
                verify_file(Path(cell[arm]["directory"]) / name, record)
                verified += 1
    for record in inventory["assets"]:
        verify_file(Path(record["path"]), record)
        verified += 1
    windows, strict_short = [], []
    for bars in (70, 205):
        cell = inventory["cells"][str(bars)]
        groups = {}
        for arm in ("full", "zero"):
            directory = Path(cell[arm]["directory"])
            summary = json.loads((directory / "occlusion_trace_summary.json").read_text())
            receipt = json.loads((directory / "receipt.json").read_text())
            metadata = summary["metadata"]
            if (summary["schema"] != "motar.occlusion-trace.v1" or summary["frame_itemsize"] != FRAME.size
                    or metadata["frame_contract"] != FRAME_CONTRACT or metadata["dt_s"] != DT
                    or receipt["status"] != "OK" or receipt["seed"] != 4501
                    or receipt["density_bars"] != bars or receipt["checkpoint_sha256"] != F_SHA):
                raise ValueError("trace/receipt identity contract differs")
            with (directory / "occlusion_frames.bin").open("rb") as stream:
                groups[arm] = selected_frames(stream, summary["frames"])
            for env, rows in groups[arm].items():
                if len(rows) != cell[arm]["first_ten_frame_counts"][str(env)]:
                    raise ValueError("selected count differs from pinned inventory")
        for env in ENVS:
            full, zero = groups["full"][env], groups["zero"][env]
            if full[0][3:6] != zero[0][3:6] or full[0][13:17] != zero[0][13:17]:
                raise ValueError("initial pursuer state differs")
            if any(full[k][17:20] != zero[k][17:20] for k in set(full) & set(zero)):
                raise ValueError("selected common target checkpoint differs")
            window = dict(frame_window(len(zero)), bars=bars, env=env, env_episode_index=0)
            windows.append(window)
            if len(zero) < CAP:
                strict_short.append({"bars": bars, "env": env, "recorded_frames": len(zero)})
    return {
        "unit": "dev08 input/runtime design v1", "selected_protocol": protocol,
        "status": "BLOCKED_STRICT600" if protocol == "strict600" else "PREFIX_INPUTS_VERIFIED",
        "strict600_status": "BLOCKED_INCOMPLETE_TARGET_TRACE", "strict600_shortfalls": strict_short,
        "true_60s_endpoint_status": "NOT_RECORDED_REQUIRES_FRAME600",
        "runtime_status": "NOT_IMPLEMENTED", "measurement_status": "NOT_RUN",
        "execution_supported": False, "launch_ready": False,
        "verified_inventory_files": verified, "layout_semantics_qualified": False,
        "inventory_sha256": INVENTORY_SHA, "configuration_source_sha256": CONFIG_SHA,
        "native_params": configuration["native_params"], "candidate_params": configuration["candidate_params"],
        "windows": windows,
        "proposed_unit": {"native_smoke": 2, "main_pairs": 20, "main_episodes": 40,
                          "total_episodes": 42, "density_order": [70, 205],
                          "pair_order": "env0..9; even env native then candidate, odd env candidate then native",
                          "smoke_in_main_denominator": False},
        "requirements": ["new additive launcher; do not call historical sitl_run.sh (global pkill/shared rootfs)",
                         "fresh per-episode external rootfs and parameter echo/readback",
                         "checkpoint termination check before command; no command at terminal/censor frame",
                         "CPU native command-prefix identity and owned-process/resource/telemetry guards",
                         "reviewed committed runtime/protocol; newly scoped SITL execution approval"],
        "limits": ["No target continuation, raw rewrite, episode dropping, GPU/SITL launch or formal S3b PASS.",
                   "Historical dev07 report steps are observation-check counts; Isaac completion steps are actions.",
                   "Censored cases stay in the planned denominator; no complete-case success/crash rate.",
                   "Endpoint times/status fields do not establish live per-step telemetry freshness or pacing.",
                   "Full/zero common-frame equality does not supersede G8 TARGET_DEPENDS_ON_PURSUER."]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", choices=("strict600", "observed-prefix-v1"), default="strict600")
    args = parser.parse_args(argv)
    try:
        result = prepare(protocol=args.protocol)
    except (OSError, ValueError, KeyError, TypeError, struct.error) as exc:
        parser.exit(2, "dev08 input contract refused: %s\n" % exc)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
