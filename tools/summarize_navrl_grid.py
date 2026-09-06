"""One-line view of any grid root: outcomes, contrasts and the per-contact mechanism columns.

Reads only committed result JSON and the contact-record sidecars; runs nothing. Works on a grid
that is still in progress, which is the point -- `python tools/summarize_navrl_grid.py <root>`
answers "where is it now" without hand-written one-off scripts.
"""

import argparse
import json
import math
import statistics
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CATEGORIES = ("vertical_out", "behind", "lateral", "no_return", "in_corridor")


def load(root):
    cells = []
    for path in sorted(Path(root).glob("*/*bars.json")):
        if path.name.endswith(("receipt.json", "manifest.json")):
            continue
        data = json.loads(path.read_text())
        data["_name"] = path.parent.name
        cells.append(data)
    return cells


def wald(a, b, field="crash_rate"):
    pa, na = a["outcome"][field], a["actual_episodes"]
    pb, nb = b["outcome"][field], b["actual_episodes"]
    delta = 100.0 * (pa - pb)
    se = 100.0 * math.sqrt(pa * (1 - pa) / na + pb * (1 - pb) / nb)
    return delta, delta - 1.96 * se, delta + 1.96 * se


def records(cell):
    path = (cell.get("contact_geometry") or {}).get("contact_records_path")
    if not path or not Path(path).is_file():
        return []
    return [json.loads(line) for line in Path(path).open()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--baseline", help="cell name to contrast every row against")
    parser.add_argument("--mechanism", action="store_true", help="add per-contact columns")
    args = parser.parse_args()

    cells = load(args.root)
    if not cells:
        raise SystemExit(f"[grid-summary] no result JSON under {args.root}")
    manifest = args.root / "cells.json"
    if manifest.is_file():
        spec = json.loads(manifest.read_text())
        print(f"# {args.root.name} | contract {spec.get('contract', 'ref5in')} | "
              f"seed {spec.get('spec', {}).get('seed')} | commit {spec.get('evaluation_commit', '')[:8]} | "
              f"{len(cells)}/{len(spec.get('spec', {}).get('cells', []))} cells")
    base = next((c for c in cells if c["_name"] == args.baseline), None) if args.baseline else None

    header = f"{'cell':<38}{'bars':>5}{'w':>6}{'crash':>9}{'capture':>9}{'timeout':>9}{'interv':>8}"
    if base is not None:
        header += f"{'Δcrash vs base':>26}"
    if args.mechanism:
        header += f"{'cont':>6}{'lat':>5}{'inc':>5}{'v(t-1)':>8}{'capbind':>9}"
    print(header)
    for cell in cells:
        condition, governor = cell["condition"], cell["speed_governor"]
        row = (f"{cell['_name']:<38}{condition['bars']:>5}"
               f"{condition.get('speed_governor_half_width_m', float('nan')):>6.2f}"
               f"{100 * cell['outcome']['crash_rate']:>8.2f}%"
               f"{100 * cell['outcome']['capture_rate']:>8.2f}%"
               f"{100 * cell['outcome']['timeout_rate']:>8.2f}%"
               f"{100 * governor['intervention_rate']:>7.1f}%")
        if base is not None:
            delta, lo, hi = wald(cell, base)
            row += f"{delta:>+9.2f} [{lo:>+6.2f},{hi:>+6.2f}]"
        if args.mechanism:
            rows = records(cell)
            lateral = sum(r["category_cmd"] == 2 for r in rows)
            in_corridor = sum(r["category_cmd"] == 4 for r in rows)
            speed = statistics.median([r["speed_act_t1"] for r in rows]) if rows else float("nan")
            binding = 100.0 * sum(r["cap_binding_t1"] for r in rows) / len(rows) if rows else float("nan")
            row += f"{len(rows):>6}{lateral:>5}{in_corridor:>5}{speed:>8.2f}{binding:>8.0f}%"
        print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
