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


def _live_training_max_epochs(default: int = 12000) -> int:
    """Read the active NavRL runner's explicit epoch ceiling when available."""
    proc = Path("/proc")
    if not proc.is_dir():
        return default
    pattern = re.compile(r"(?:^|\s)--max_epochs\s+(\d+)(?:\s|$)")
    for cmdline_path in proc.glob("[0-9]*/cmdline"):
        try:
            command = cmdline_path.read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except (OSError, PermissionError):
            continue
        if "runner.py" not in command or "--task navrl_task" not in command or "--train" not in command:
            continue
        if match := pattern.search(command):
            return int(match.group(1))
    return default


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


def _active_record(
    summary: Dict[str, Any], csv_path: Path, *, max_epochs: int
) -> Dict[str, Any]:
    rows = _load_rows(csv_path)
    tail = rows[-50:]
    age_min = max(0.0, (datetime.now(timezone.utc).timestamp() - csv_path.stat().st_mtime) / 60.0)
    return {
        "run": summary["run"],
        "is_live": True,
        "metrics_age_min": age_min,
        "epoch": summary["last_epoch"],
        "max_epochs": max_epochs,
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


def _live_density_promotions() -> List[Dict[str, Any]]:
    live_link = RL_ROOT / "train_session_logs/current_training.log"
    if not live_link.exists():
        return []
    pattern = re.compile(
        r"density curriculum promoted \| bars (\d+) -> (\d+) "
        r"after (\d+) eps, capture=([0-9.]+)"
    )
    promotions = []
    for source, target, episodes, capture in pattern.findall(
        live_link.read_text(encoding="utf-8", errors="ignore")
    ):
        promotions.append(
            {
                "source": int(source),
                "target": int(target),
                "episodes": int(episodes),
                "capture": float(capture),
            }
        )
    return promotions


def _v2_search_update(active: Dict[str, Any]) -> Dict[str, Any]:
    promotions = _live_density_promotions()
    bars = int(round(active.get("n_bars_active") or 70))
    capture_tail = active.get("captured_rate")
    comparison = [
        {
            "label": f"{item['source']} → {item['target']} promotion",
            "bars": item["target"],
            "capture": item["capture"],
            "unique": None,
            "verdict": f"PASS over {item['episodes']:,} episodes",
        }
        for item in promotions
    ]
    comparison.append(
        {
            "label": "current stage · rolling tail",
            "bars": bars,
            "capture": capture_tail,
            "unique": None,
            "verdict": "live diagnostic only; not the 16,384-episode promotion gate",
        }
    )
    promotion_text = " → ".join(
        [str(promotions[0]["source"])] + [str(item["target"]) for item in promotions]
    ) if promotions else str(bars)
    gate_captures = ", ".join(f"{item['capture'] * 100:.1f}%" for item in promotions)
    tail_text = f"{capture_tail * 100:.1f}%" if capture_tail is not None else "pending"
    return {
        "subtitle": "2026-07-31 · v2 search-arena density curriculum · running snapshot",
        "headline": f"Task-v2 training is live at {bars} bars after {len(promotions)} promotions.",
        "summary": (
            f"The 40 m search-arena run has advanced {promotion_text}; completed promotion-window "
            f"capture values are {gate_captures or 'not yet available'}. The current 50-epoch tail "
            f"is {tail_text}, which is diagnostic only and must not be mistaken for the 16,384-episode "
            "gate. Training may continue, but checkpoint evaluation remains pending the provenance "
            "and v2 gate fixes identified by the independent audit."
        ),
        "active_experiment": {
            **active,
            "bars": bars,
            "selector": "cluster_sector",
            "cluster_gap_m": 0.45,
            "sectors": 8,
            "arena_xy_m": 40,
            "arena_z_m": 3,
            "density_final": 300,
            "density_step": 15,
            "density_threshold": 0.70,
            "density_window_episodes": 16384,
        },
        "milestones": [
            {
                "label": "DENSITY",
                "value": f"{bars} / 300",
                "detail": f"self-paced +15 bars; chain {promotion_text}",
                "state": "active",
            },
            {
                "label": "PROMOTIONS",
                "value": str(len(promotions)),
                "detail": f"window capture {gate_captures or 'pending'}",
                "state": "pass" if promotions else "active",
            },
            {
                "label": "PROVENANCE",
                "value": "6 / 7",
                "detail": "arena contract saved; placement_touch is still missing",
                "state": "warn",
            },
            {
                "label": "1650 Ti · 4GB",
                "value": "CONDITIONAL",
                "detail": "64-env path fits the batch; real-card 8-epoch smoke still required",
                "state": "warn",
            },
        ],
        "comparison": comparison,
        "gates": [
            {"label": "density curriculum", "value": f"RUNNING · {promotion_text}"},
            {"label": "collapse safety", "value": "PASS · reward guard off, NaN/Inf fail-fast on"},
            {"label": "evaluation contract", "value": "PATCH BEFORE EVAL · touch/z/gap coverage incomplete"},
            {"label": "1650 Ti", "value": "SMOKE REQUIRED · recommend free VRAM ≥3.6–3.7 GiB"},
        ],
        "decision": (
            "Keep the current 128-env training process running. Do not interpret the rolling tail as "
            "a promotion decision. Before held-out evaluation, add placement_touch to checkpoint "
            "provenance, enforce z/gap/touch in the v2 evaluator, and repair NAVRL_V2_FORCE ordering. "
            "Treat the 4GB launcher as provisional until it passes an actual 1650 Ti smoke run."
        ),
    }


def _research_update(
    active: Optional[Dict[str, Any]], latest: Optional[Dict[str, Any]]
) -> Dict[str, Any]:
    if active and "v2-search" in active.get("run", ""):
        return _v2_search_update(active)
    record = active or latest or {}
    experiment = {
        "is_live": bool(active),
        "max_epochs": 13800,
        "bars": int(record.get("n_bars_active") or record.get("last_n_bars_active") or 100),
        "run": record.get(
            "run", "ppo_260731_0343_navrl_corridor6-fixed100-s1"
        ),
        "epoch": record.get("epoch") if active else record.get("last_epoch", 13800),
        "capture_tail": active["captured_rate"] if active else None,
        "crash_tail": active["crash_rate"] if active else None,
        "selector": "cluster_sector",
        "corridor_tokens": 6,
        "cluster_gap_m": 0.45,
        "sectors": 8,
        "unique_bars": 4.9,
        "duplicate_tokens": 0.2,
    }
    return {
        "subtitle": "2026-07-31 · corridor-token fixed-100 A/B complete",
        "headline": "Corridor tokens reduced bar contact, but missed the advancement gate.",
        "summary": (
            "The K=6 free-gap representation passed its physical geometry and checkpoint-schema "
            "tests, then completed an 800-epoch fixed-100 adaptation. On 4,003 held-out episodes, "
            "ep13800 reached 66.10% capture versus 64.53% for ep12500 (+1.57 percentage points; "
            "95% CI [-0.51,+3.66]) while bar contact fell from 33.18% to 32.25%. The collision "
            "signal is encouraging, but the preregistered 68% and +3pp gates were not met."
        ),
        "active_experiment": experiment,
        "milestones": [
            {
                "label": "GEOMETRY",
                "value": "PASS",
                "detail": "center clear 100%; bound-on-bar 97.8%; width sanity 98.8%",
                "state": "pass",
            },
            {
                "label": "OBS SCHEMA",
                "value": "898 → 946",
                "detail": "explicit six-corridor append and contract-safe checkpoint expansion",
                "state": "pass",
            },
            {
                "label": "HELD-OUT CAPTURE",
                "value": "66.10%",
                "detail": "4,003 episodes at 100 bars; pilot target was 68%",
                "state": "warn",
            },
            {
                "label": "BAR CONTACT",
                "value": "32.25%",
                "detail": "down from the immutable ep12500 baseline of 33.18%",
                "state": "pass",
            },
        ],
        "comparison": [
            {
                "label": "ep12500 · cluster-sector baseline",
                "bars": 100,
                "capture": 0.6453,
                "unique": 4.9,
                "verdict": "immutable baseline; bar contact 33.18%",
            },
            {
                "label": "ep13100 · corridor screen",
                "bars": 100,
                "capture": 0.6420,
                "unique": 4.9,
                "verdict": "no early gain; bar contact 33.75%",
            },
            {
                "label": "ep13450 · corridor screen",
                "bars": 100,
                "capture": 0.5993,
                "unique": 4.9,
                "verdict": "mid-run regression; bar contact 38.15%",
            },
            {
                "label": "ep13800 · corridor confirm",
                "bars": 100,
                "capture": 0.6610,
                "unique": 4.9,
                "verdict": "4,003 episodes; bar contact 32.25%",
            },
        ],
        "gates": [
            {"label": "capture target", "value": "FAIL · 66.10% < 68%"},
            {"label": "minimum gain", "value": "FAIL · +1.57pp < +3pp"},
            {"label": "bar contact", "value": "PASS · 32.25% < 33.18%"},
            {"label": "significance", "value": "INCONCLUSIVE · 95% CI crosses zero"},
        ],
        "decision": (
            "Freeze corridor6 as a partial/negative result and do not buy more epochs with the same "
            "representation. Keep density=100 and goal distance=4..16 fixed for the next diagnostic, "
            "then test whether two depth layers per sector preserve the front/back surfaces that one "
            "corridor token compresses away."
        ),
    }


def _corridor_token_plan() -> Dict[str, Any]:
    return {
        "title": "Corridor token",
        "subtitle": "completed pilot · physical geometry passed, policy gate failed",
        "definition": (
            "A corridor token is a compact description of one locally traversable opening between "
            "obstacles. The current obstacle token says where a bar surface is; a corridor token says "
            "where the drone may fit, how wide the opening is, and how far that opening remains clear."
        ),
        "why_now": (
            "The geometry was real and the actor consumed it safely: K=6 expanded the observation "
            "from 898 to 946 dimensions without changing historical offsets. It reduced bar contact "
            "by 0.93pp, but capture improved only 1.57pp and its confidence interval crossed zero. "
            "That rules out simply extending this same run; the next representation must preserve "
            "more depth structure rather than add more copies of the same local gap summary."
        ),
        "current": {
            "label": "Obstacle token · current",
            "question": "Where are the nearest bars?",
            "fields": ["bearing", "range", "relative geometry", "valid mask"],
            "weakness": "The policy must infer the opening between bars and whether the drone fits.",
        },
        "proposed": {
            "label": "Corridor token · tested",
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
                "detail": "Physical probe passed: center clearance 100%, bar-boundary 97.8%, width sanity 98.8%.",
                "state": "done",
            },
            {
                "id": "P2",
                "title": "Contract-safe input",
                "detail": "Expanded 898→946 dimensions, selectively initialized the new projection, and recorded schema provenance.",
                "state": "done",
            },
            {
                "id": "P3",
                "title": "Short fixed-100 A/B",
                "detail": "Completed 800 epochs and 4,003 held-out episodes: 66.10% capture, 32.25% bar contact.",
                "state": "done",
            },
        ],
        "pilot_gate": (
            "FAIL: capture 66.10% < 68% and gain +1.57pp < +3pp; bar-contact reduction passed. "
            "Do not extend corridor6. Advance to a separately gated two-depth-layer diagnostic."
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
        active = _active_record(
            active_summary,
            active_path,
            max_epochs=_live_training_max_epochs(),
        )

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
