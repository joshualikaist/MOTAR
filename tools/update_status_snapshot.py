#!/usr/bin/env python3
"""Synchronize the static research dashboard with local NavRL run evidence.

The dashboard has two data sources by design: ``status.json`` for HTTP hosting and an inline
fallback in ``index.html`` for direct/offline viewing. This tool always writes both from the same
object so they cannot silently drift.
"""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
from statistics import fmean
from typing import Any, Dict, Iterable, List, Optional


ROOT = Path(__file__).resolve().parents[1]
RL_ROOT = ROOT / "aerial_gym/rl_training/rl_games"
RUNS_ROOT = RL_ROOT / "runs"
STATUS_PATH = ROOT / "docs/status/status.json"
HTML_PATH = ROOT / "docs/status/index.html"
CORRECTED_CURVE_PATH = ROOT / "results/corrected_chirality_density_curve.csv"


def _float(row: Dict[str, str], key: str) -> Optional[float]:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _int(row: Dict[str, str], key: str) -> Optional[int]:
    value = _float(row, key)
    return int(value) if value is not None else None


def _iso_mtime(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat()


def _mean(rows: Iterable[Dict[str, str]], key: str) -> Optional[float]:
    values = [value for row in rows if (value := _float(row, key)) is not None]
    return fmean(values) if values else None


def _training_process_exists() -> bool:
    proc = Path("/proc")
    if not proc.is_dir():
        return False
    for cmdline_path in proc.glob("[0-9]*/cmdline"):
        try:
            command = cmdline_path.read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except (OSError, PermissionError):
            continue
        if "runner.py" in command and "--task navrl_task" in command and "--train" in command:
            return True
    return False


def _load_rows(csv_path: Path) -> List[Dict[str, str]]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _summarize_run(csv_path: Path, *, is_active: bool) -> Dict[str, Any]:
    run_dir = csv_path.parents[1]
    rows = _load_rows(csv_path)
    if not rows:
        raise ValueError(f"empty metrics CSV: {csv_path}")
    last = rows[-1]
    peak_capture = max(rows, key=lambda row: _float(row, "captured_rate") or -math.inf)
    peak_reward = max(rows, key=lambda row: _float(row, "mean_reward") or -math.inf)
    summary_path = run_dir / "aerial_run/run_summary.json"
    saved = {}
    if summary_path.is_file():
        saved = json.loads(summary_path.read_text(encoding="utf-8"))

    finalized_at = None if is_active else saved.get("finalized_at", _iso_mtime(csv_path))
    summary = {
        "run": run_dir.name,
        "finalized_at": finalized_at,
        "exit_reason": "running" if is_active else saved.get("exit_reason", "interrupted"),
        "epochs_logged": len(rows),
        "first_epoch": _int(rows[0], "epoch"),
        "last_epoch": _int(last, "epoch"),
        "is_navrl": True,
        "last_mean_reward": _float(last, "mean_reward"),
        "last_mean_episode_length": _float(last, "mean_episode_length"),
        "last_captured_rate": _float(last, "captured_rate"),
        "last_crash_rate": _float(last, "crash_rate"),
        "last_timeout_rate": _float(last, "timeout_rate"),
        "last_closest_approach_m": _float(last, "closest_approach_m"),
        "last_closest_min_m": _float(last, "closest_min_m"),
        "last_curriculum_max_m": _float(last, "curriculum_max_m"),
        "last_n_bars_active": _int(last, "n_bars_active"),
        "peak_captured_rate": _float(peak_capture, "captured_rate"),
        "peak_captured_epoch": _int(peak_capture, "epoch"),
        "peak_mean_reward": _float(peak_reward, "mean_reward"),
        "peak_epoch": _int(peak_reward, "epoch"),
        "reward_collapse": bool(saved.get("reward_collapse", False)),
    }
    # The generic peak-relative guard was disabled for density curricula, but this run is a
    # measured PPO collapse rather than a normal promotion drop.
    if run_dir.name.startswith("ppo_260730_1154_"):
        summary["reward_collapse"] = True
        summary["collapse_detail"] = (
            "KL crossed 0.04 at epoch 10276; latent means exploded and tail500 capture fell to 1.0%."
        )
    return summary


def _latest_barprobe() -> Dict[str, Optional[float]]:
    live_link = RL_ROOT / "train_session_logs/current_training.log"
    if not live_link.exists():
        return {"unique": None, "duplicate": None}
    pattern = re.compile(r"unique=([0-9.]+) duplicate=([0-9.]+)")
    matches = pattern.findall(live_link.read_text(encoding="utf-8", errors="ignore"))
    if not matches:
        return {"unique": None, "duplicate": None}
    unique, duplicate = matches[-1]
    return {"unique": float(unique), "duplicate": float(duplicate)}


def _corrected_density_curve() -> Dict[str, Any]:
    rows = []
    with CORRECTED_CURVE_PATH.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append({key: float(value) for key, value in row.items()})
    return {
        "notes": [
            "Corrected bearing/chirality + bounded squashed-Gaussian backbone.",
            "Deterministic held-out evaluation of the 85-bar training policy; ~2049 episodes/cell.",
            "85-bar training plateau is 0.676 +/- 0.001; the 0.689 table cell is held-out evaluation.",
            "65-bar predecessor plateau was 0.678: equal competence at +31% training density.",
            "This curve used greedy +/-10 deg suppression. The active cluster-sector selector is a same-shape ablation and is not in this curve.",
        ],
        "rows": rows,
        "mtime": _iso_mtime(CORRECTED_CURVE_PATH),
    }


def _active_record(summary: Dict[str, Any], csv_path: Path) -> Dict[str, Any]:
    rows = _load_rows(csv_path)
    tail = rows[-50:]
    age_min = max(0.0, (datetime.now(timezone.utc).timestamp() - csv_path.stat().st_mtime) / 60.0)
    return {
        "run": summary["run"],
        "is_live": True,
        "metrics_age_min": age_min,
        "epoch": summary["last_epoch"],
        "max_epochs": 12000,
        "epochs_logged": summary["epochs_logged"],
        "tail_epochs": len(tail),
        "captured_rate": _mean(tail, "captured_rate"),
        "crash_rate": _mean(tail, "crash_rate"),
        "timeout_rate": _mean(tail, "timeout_rate"),
        "mean_reward": _mean(tail, "mean_reward"),
        "closest_approach_m": _mean(tail, "closest_approach_m"),
        "n_bars_active": _mean(tail, "n_bars_active"),
        "curriculum_max_m": _mean(tail, "curriculum_max_m"),
    }


def _research_update(
    active: Optional[Dict[str, Any]], latest: Optional[Dict[str, Any]]
) -> Dict[str, Any]:
    record = active or latest or {}
    experiment = {
        "is_live": bool(active),
        "max_epochs": 13000,
        "bars": int(record.get("n_bars_active") or record.get("last_n_bars_active") or 100),
        "run": record.get(
            "run", "ppo_260731_0226_navrl_fix3-low-lr-replay-b5-100bars-s1"
        ),
        "epoch": record.get("epoch") if active else record.get("last_epoch", 13000),
        "capture_tail": active["captured_rate"] if active else None,
        "crash_tail": active["crash_rate"] if active else None,
        "selector": "cluster_sector",
        "cluster_gap_m": 0.45,
        "sectors": 8,
        "unique_bars": 4.9,
        "duplicate_tokens": 0.2,
    }
    return {
        "subtitle": "2026-07-31 · FIX3 target motion + low-LR density replay",
        "headline": "100-bar replay stayed stable, but did not clear the held-out 70% gate.",
        "summary": (
            "FIX3 removed target heading oscillation and the 500-epoch replay kept PPO updates "
            "well behaved (|KL|max 0.00232). On the fixed 100-bar, four-speed held-out grid, "
            "ep13000 reached 65.63% capture versus 64.53% for the unadapted ep12500 checkpoint: "
            "+1.10 percentage points, but the 95% interval [-0.99, +3.19] crosses zero. More of the "
            "same replay is therefore unlikely to buy the missing performance."
        ),
        "active_experiment": experiment,
        "milestones": [
            {
                "label": "TARGET MOTION",
                "value": "FIX3 parity",
                "detail": "Python and browser use the same heading-continuity contract",
                "state": "pass",
            },
            {
                "label": "UPDATE SAFETY",
                "value": "|KL|max 0.00232",
                "detail": "low-LR replay remained far below the 0.04 fail-stop",
                "state": "pass",
            },
            {
                "label": "HELD-OUT CAPTURE",
                "value": "65.63%",
                "detail": "ep13000 at 100 bars; 70% target remains unmet",
                "state": "warn",
            },
            {
                "label": "FAILURE MODE",
                "value": "95.5% bar contact",
                "detail": "of observed crashes; below-ground failures remain small",
                "state": "fail",
            },
        ],
        "comparison": [
            {
                "label": "ep12500 · FIX3, no replay",
                "bars": 100,
                "capture": 0.6453,
                "unique": 4.9,
                "verdict": "warm-start baseline; bar contact 33.18%",
            },
            {
                "label": "ep12850 · replay",
                "bars": 100,
                "capture": 0.6403,
                "unique": 4.9,
                "verdict": "no gain; bar contact 34.20%",
            },
            {
                "label": "ep13000 · replay",
                "bars": 100,
                "capture": 0.6563,
                "unique": 4.9,
                "verdict": "best observed; bar contact 32.23%",
            },
        ],
        "gates": [
            {"label": "task contract", "value": "PASS · FIX3 target motion + browser parity"},
            {"label": "update safety", "value": "PASS · |KL|max 0.00232"},
            {"label": "held-out target", "value": "FAIL · 65.63% < 70%"},
            {"label": "significance", "value": "NO · +1.10pp, CI crosses zero"},
        ],
        "decision": (
            "Stop extending the low-LR replay. Preserve ep12500 and ep13000 as baselines, validate a "
            "free-space corridor extractor without changing policy input, then run a short fixed-100 "
            "A/B with an explicit new observation schema. Do not silently reinterpret the current "
            "898-D input."
        ),
    }


def _corridor_token_plan() -> Dict[str, Any]:
    return {
        "title": "Corridor token",
        "subtitle": "next ablation · represent traversable gaps, not only obstacle surfaces",
        "definition": (
            "A corridor token is a compact description of one locally traversable opening between "
            "obstacles. The current obstacle token says where a bar surface is; a corridor token says "
            "where the drone may fit, how wide the opening is, and how far that opening remains clear."
        ),
        "why_now": (
            "At 100 bars the cluster-sector encoder already associates about 4.9 unique bars out of "
            "8 slots, hit_token_given_fov is 0.839, and duplicate use is only 0.2. Yet most crashes "
            "are bar contacts. The next bottleneck is therefore converting detected surfaces into a "
            "safe passage affordance, not simply adding more obstacle hits."
        ),
        "current": {
            "label": "Obstacle token · current",
            "question": "Where are the nearest bars?",
            "fields": ["bearing", "range", "relative geometry", "valid mask"],
            "weakness": "The policy must infer the opening between bars and whether the drone fits.",
        },
        "proposed": {
            "label": "Corridor token · proposed",
            "question": "Where can the drone pass?",
            "fields": [
                "gap center bearing",
                "usable width",
                "left/right clearance",
                "clear depth or TTC",
                "valid mask",
            ],
            "weakness": "Local affordance only; it is not a full planner and still needs policy choice.",
        },
        "steps": [
            {
                "id": "P0",
                "title": "Freeze evidence",
                "detail": "Keep ep12500 and ep13000 plus the fixed 100-bar four-speed evaluation as immutable baselines.",
                "state": "done",
            },
            {
                "id": "P1",
                "title": "Geometry-only diagnostic",
                "detail": "Extract gap center, usable width, and clear depth from the existing LiDAR, but do not feed it to the actor yet.",
                "state": "now",
            },
            {
                "id": "P2",
                "title": "Contract-safe input",
                "detail": "Add explicit corridor fields and selectively initialize the widened input projection; record the observation schema in checkpoints.",
                "state": "todo",
            },
            {
                "id": "P3",
                "title": "Short fixed-100 A/B",
                "detail": "Compare the unchanged cluster-sector baseline against corridor tokens under the same seeds and held-out sweep.",
                "state": "todo",
            },
        ],
        "pilot_gate": (
            "Advance only if the fixed-100 aggregate reaches at least 68%, improves by at least "
            "3 percentage points over ep12500, and lowers bar-contact rate. Treat 70% with a "
            "replicated seed as the backbone-freeze gate."
        ),
    }


def build_snapshot() -> Dict[str, Any]:
    status = json.loads(STATUS_PATH.read_text(encoding="utf-8"))
    csv_paths = sorted(RUNS_ROOT.glob("*/aerial_run/epoch_metrics.csv"))
    training_live = _training_process_exists()
    active_path = None
    if training_live:
        unfinished = [
            path for path in csv_paths if not (path.parent / "run_summary.json").is_file()
        ]
        if unfinished:
            active_path = max(unfinished, key=lambda path: path.stat().st_mtime)

    summaries = []
    for path in csv_paths:
        summaries.append(_summarize_run(path, is_active=(path == active_path)))
    summaries.sort(key=lambda item: item["run"])

    active = None
    if active_path is not None:
        active_summary = next(item for item in summaries if item["run"] == active_path.parents[1].name)
        active = _active_record(active_summary, active_path)

    finalized = [item for item in summaries if item["run"] != (active or {}).get("run")]
    latest = max(
        finalized,
        key=lambda item: (item.get("finalized_at") or "", item["run"]),
        default=None,
    )
    status.update(
        {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "repo": str(ROOT),
            "n_runs": len(summaries),
            "active_run": active,
            "latest_run": latest,
            "runs": summaries,
            "research_update": _research_update(active, latest),
            "corridor_token": _corridor_token_plan(),
        }
    )
    status.setdefault("density_curves", {})[
        "corrected_chirality_density_curve"
    ] = _corrected_density_curve()
    return status


def write_snapshot(status: Dict[str, Any]) -> None:
    STATUS_PATH.write_text(
        json.dumps(status, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    html = HTML_PATH.read_text(encoding="utf-8")
    compact = json.dumps(status, ensure_ascii=False, separators=(",", ":"))
    pattern = re.compile(
        r'(<script id="fallback" type="application/json">).*?(</script>)',
        flags=re.DOTALL,
    )
    html, count = pattern.subn(rf"\g<1>{compact}\g<2>", html, count=1)
    if count != 1:
        raise RuntimeError("could not locate exactly one dashboard fallback JSON block")
    HTML_PATH.write_text(html, encoding="utf-8")


def main() -> int:
    status = build_snapshot()
    write_snapshot(status)
    active = status.get("active_run")
    print(
        "[status] synchronized "
        f"{status['n_runs']} runs; active={active['run'] if active else 'none'}; "
        f"generated_at={status['generated_at']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
