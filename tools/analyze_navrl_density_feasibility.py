#!/usr/bin/env python3
"""Monte-Carlo geometry audit for the 24x24 m NavRL bar field.

This deliberately ignores the learned policy. It asks a more basic question: after inflating each
bar by the 0.28 m drone footprint and a requested side-clearance margin, does *any* 2-D path still
connect the spawn side to the target side? Speed cannot repair a disconnected free-space map.
"""

import argparse
from collections import deque
from pathlib import Path
import random
import re

import numpy as np


ARENA = 24.0
BAND = (0.13 * ARENA, 0.96 * ARENA, 0.0, ARENA)
DRONE_HALF = 0.28 * 0.5


def load_bar_sizes(root):
    sizes = []
    pattern = re.compile(r'<box size="([0-9.]+) ([0-9.]+) 2[.]0000"')
    for path in sorted((root / "resources/models/environment_assets/bars").glob("*.urdf")):
        match = pattern.search(path.read_text())
        if match:
            sizes.append((float(match.group(1)), float(match.group(2))))
    if not sizes:
        raise RuntimeError("bar URDF sizes not found")
    return sizes


def place_bars(count, rng, sizes):
    x0, x1, y0, y1 = BAND
    placed = []
    min_spacing = 1.5
    attempts = 0
    relaxations = 0
    for _ in range(count):
        while True:
            candidates = [
                (rng.uniform(x0, x1), rng.uniform(y0, y1)) for _ in range(32)
            ]
            valid = []
            for x, y in candidates:
                valid.append(
                    all((x - px) ** 2 + (y - py) ** 2 >= min_spacing**2 for px, py, _, _ in placed)
                )
            if any(valid):
                idx = valid.index(True)
                w, d = rng.choice(sizes)
                placed.append((candidates[idx][0], candidates[idx][1], w, d))
                break
            attempts += 32
            if attempts >= 128:
                min_spacing *= 0.8
                attempts = 0
                relaxations += 1
    return placed, min_spacing, relaxations


def crossing_metrics(bars, resolution, side_clearance):
    nx = int(round(ARENA / resolution)) + 1
    ny = nx
    occupied = np.zeros((nx, ny), dtype=bool)
    inflation = DRONE_HALF + side_clearance
    for x, y, w, d in bars:
        xa = max(0, int(np.floor((x - w * 0.5 - inflation) / resolution)))
        xb = min(nx - 1, int(np.ceil((x + w * 0.5 + inflation) / resolution)))
        ya = max(0, int(np.floor((y - d * 0.5 - inflation) / resolution)))
        yb = min(ny - 1, int(np.ceil((y + d * 0.5 + inflation) / resolution)))
        occupied[xa : xb + 1, ya : yb + 1] = True

    start_x = int(round(1.0 / resolution))
    goal_x = int(round(23.5 / resolution))
    queue = deque()
    seen = np.zeros_like(occupied)
    for y in np.flatnonzero(~occupied[start_x]):
        queue.append((start_x, int(y)))
        seen[start_x, y] = True
    neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    while queue:
        x, y = queue.popleft()
        for dx, dy in neighbors:
            xx, yy = x + dx, y + dy
            if 0 <= xx < nx and 0 <= yy < ny and not occupied[xx, yy] and not seen[xx, yy]:
                seen[xx, yy] = True
                queue.append((xx, yy))
    goal_free = ~occupied[goal_x]
    reachable_goal = seen[goal_x] & goal_free
    fraction = float(reachable_goal.sum()) / max(1, int(goal_free.sum()))
    return bool(reachable_goal.any()), fraction


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--densities", nargs="+", type=int, default=[75, 110, 130, 150])
    parser.add_argument("--margins", nargs="+", type=float, default=[0.0, 0.1, 0.2])
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--resolution", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    sizes = load_bar_sizes(root)
    print("density,side_clearance_m,path_exists_rate,reachable_goal_fraction,placement_relaxed_rate,mean_final_spacing_m")
    for density in args.densities:
        outcomes = {margin: [] for margin in args.margins}
        goal_fractions = {margin: [] for margin in args.margins}
        relaxed = []
        spacing = []
        for trial in range(args.trials):
            rng = random.Random(args.seed + density * 100003 + trial)
            bars, final_spacing, n_relax = place_bars(density, rng, sizes)
            relaxed.append(n_relax > 0)
            spacing.append(final_spacing)
            for margin in args.margins:
                exists, fraction = crossing_metrics(bars, args.resolution, margin)
                outcomes[margin].append(exists)
                goal_fractions[margin].append(fraction)
        for margin in args.margins:
            print(
                "%d,%.2f,%.3f,%.3f,%.3f,%.3f"
                % (
                    density,
                    margin,
                    np.mean(outcomes[margin]),
                    np.mean(goal_fractions[margin]),
                    np.mean(relaxed),
                    np.mean(spacing),
                )
            )

    print("\nStopping-distance reference (a=2 m/s^2, control+perception latency=0.1 s):")
    for speed in (0.5, 1.0, 1.5, 2.0):
        stopping = speed * 0.1 + speed * speed / (2.0 * 2.0)
        print("  %.1f m/s -> %.3f m" % (speed, stopping))


if __name__ == "__main__":
    main()
