/*
 * Browser-only mirror of target_route_planner.py's deterministic geometry contract.
 * Source receipt (development WIP inspected 2026-08-25):
 *   topology branch fcec3d2, target_route_planner.py SHA-256
 *   99ff8fe8852595dfc11fb52091c219fbf39da792b67167e229a6a01b131a465b
 *
 * This module explains the target's global route. It is NOT the pursuer PPO, PhysX, or a
 * performance measurement. Keep it DOM/THREE independent so Node can audit geometry exactly.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.NavRLArenaRoute = api;
})(typeof window !== 'undefined' ? window : this, function () {
  'use strict';

  const CONTRACT = Object.freeze({
    resolutionM: 0.25,
    trackingMarginM: 0.45,
    boundaryMarginM: 1.25,
    maxExpansions: 50000,
    maxWaypoints: 128,
    goalToleranceM: 0.05,
    minGoalDistanceM: 6.0,
    physicalBoxXYZ: Object.freeze([0.28, 0.28, 0.12]),
  });
  const NEIGHBORS = Object.freeze([
    [-1, 0, 1], [0, -1, 1], [0, 1, 1], [1, 0, 1],
    [-1, -1, Math.SQRT2], [-1, 1, Math.SQRT2],
    [1, -1, Math.SQRT2], [1, 1, Math.SQRT2],
  ]);

  function conservativeXYSupportFromBox(boxXYZ) {
    if (!Array.isArray(boxXYZ) || boxXYZ.length !== 3
        || boxXYZ.some(function (v) { return !Number.isFinite(v) || v <= 0; })) {
      throw new Error('physical target box must contain three finite positive sizes');
    }
    const radius = 0.5 * Math.hypot(boxXYZ[0], boxXYZ[1], boxXYZ[2]);
    return {x: radius, y: radius};
  }

  function finitePoint(value) {
    return value && Number.isFinite(value.x) && Number.isFinite(value.y);
  }

  // Exact slab test: touching a closed inflated AABB is unsafe.
  function segmentIntersectsClosedAABB(p0, p1, lo, hi) {
    const direction = {x: p1.x - p0.x, y: p1.y - p0.y};
    let enter = 0, leave = 1;
    for (const axis of ['x', 'y']) {
      if (Math.abs(direction[axis]) <= 1e-12) {
        if (p0[axis] < lo[axis] || p0[axis] > hi[axis]) return false;
        continue;
      }
      let a = (lo[axis] - p0[axis]) / direction[axis];
      let b = (hi[axis] - p0[axis]) / direction[axis];
      if (a > b) { const temporary = a; a = b; b = temporary; }
      enter = Math.max(enter, a);
      leave = Math.min(leave, b);
      if (enter > leave) return false;
    }
    return enter <= leave && leave >= 0 && enter <= 1;
  }

  function segmentIsSafe(p0, p1, admissibleLo, admissibleHi, bars, inflatedHalf) {
    if (![p0, p1, admissibleLo, admissibleHi].every(finitePoint)) return false;
    if (admissibleHi.x <= admissibleLo.x || admissibleHi.y <= admissibleLo.y) return false;
    // Strict boundary inclusion mirrors Python: equality is unsafe.
    for (const point of [p0, p1]) {
      if (point.x <= admissibleLo.x || point.x >= admissibleHi.x
          || point.y <= admissibleLo.y || point.y >= admissibleHi.y) return false;
    }
    if (!Array.isArray(bars) || !Array.isArray(inflatedHalf)
        || bars.length !== inflatedHalf.length) return false;
    for (let index = 0; index < bars.length; index++) {
      const center = bars[index], half = inflatedHalf[index];
      if (!finitePoint(center) || !finitePoint(half) || half.x < 0 || half.y < 0) return false;
      if (segmentIntersectsClosedAABB(
        p0, p1,
        {x: center.x - half.x, y: center.y - half.y},
        {x: center.x + half.x, y: center.y + half.y}
      )) return false;
    }
    return true;
  }

  class MinHeap {
    constructor(compare) { this.values = []; this.compare = compare; }
    get length() { return this.values.length; }
    push(value) {
      const a = this.values; a.push(value);
      let index = a.length - 1;
      while (index > 0) {
        const parent = Math.floor((index - 1) / 2);
        if (this.compare(a[parent], value) <= 0) break;
        a[index] = a[parent]; index = parent;
      }
      a[index] = value;
    }
    pop() {
      const a = this.values;
      if (!a.length) return undefined;
      const root = a[0], tail = a.pop();
      if (a.length) {
        let index = 0;
        while (true) {
          const left = 2 * index + 1, right = left + 1;
          if (left >= a.length) break;
          let child = left;
          if (right < a.length && this.compare(a[right], a[left]) < 0) child = right;
          if (this.compare(a[child], tail) >= 0) break;
          a[index] = a[child]; index = child;
        }
        a[index] = tail;
      }
      return root;
    }
  }

  function tupleCompare(a, b) {
    for (let index = 0; index < a.length; index++) {
      if (a[index] < b[index]) return -1;
      if (a[index] > b[index]) return 1;
    }
    return 0;
  }

  function emptyPlan(status, expanded) {
    return {
      status: status, valid: false, waypoints: [], expandedNodes: expanded || 0,
      rawGridNodes: 0, smoothedNodes: 0, pathLengthM: 0,
    };
  }

  function grid(arenaLo, arenaHi, resolution) {
    const shapeX = Math.max(1, Math.floor((arenaHi.x - arenaLo.x) / resolution));
    const shapeY = Math.max(1, Math.floor((arenaHi.y - arenaLo.y) / resolution));
    const axisX = new Float64Array(shapeX), axisY = new Float64Array(shapeY);
    for (let i = 0; i < shapeX; i++) axisX[i] = arenaLo.x + (i + 0.5) * resolution;
    for (let j = 0; j < shapeY; j++) axisY[j] = arenaLo.y + (j + 0.5) * resolution;
    return {axisX, axisY, shapeX, shapeY};
  }

  function nearestIndex(axis, value) {
    // np.argmin keeps the first (lower-index) cell on an exact distance tie.
    let best = 0, bestDistance = Infinity;
    for (let index = 0; index < axis.length; index++) {
      const distance = Math.abs(axis[index] - value);
      if (distance < bestDistance) { best = index; bestDistance = distance; }
    }
    return best;
  }

  function cell(point, mesh) {
    return [nearestIndex(mesh.axisX, point.x), nearestIndex(mesh.axisY, point.y)];
  }

  function indexOf(i, j, shapeY) { return i * shapeY + j; }

  function inflatedGeometry(bars, support, tracking) {
    if (!Array.isArray(bars)) return null;
    const centers = [], half = [];
    for (const bar of bars) {
      const widthX = bar && bar.w != null ? Number(bar.w) : Number(bar && bar.widthX);
      const widthY = bar && bar.h != null ? Number(bar.h)
        : (bar && bar.widthY != null ? Number(bar.widthY) : widthX);
      if (!finitePoint(bar) || !Number.isFinite(widthX) || !Number.isFinite(widthY)
          || widthX < 0 || widthY < 0) return null;
      centers.push({x: bar.x, y: bar.y});
      half.push({x: 0.5 * widthX + support.x + tracking,
        y: 0.5 * widthY + support.y + tracking});
    }
    return {centers, half};
  }

  function occupancy(mesh, admissibleLo, admissibleHi, geometry, cellPad) {
    const free = new Uint8Array(mesh.shapeX * mesh.shapeY);
    for (let i = 0; i < mesh.shapeX; i++) {
      const x = mesh.axisX[i];
      for (let j = 0; j < mesh.shapeY; j++) {
        const y = mesh.axisY[j];
        let clear = x > admissibleLo.x && x < admissibleHi.x
          && y > admissibleLo.y && y < admissibleHi.y;
        if (clear) {
          for (let k = 0; k < geometry.centers.length; k++) {
            if (Math.abs(x - geometry.centers[k].x) <= geometry.half[k].x + cellPad
                && Math.abs(y - geometry.centers[k].y) <= geometry.half[k].y + cellPad) {
              clear = false; break;
            }
          }
        }
        free[indexOf(i, j, mesh.shapeY)] = clear ? 1 : 0;
      }
    }
    return free;
  }

  function anchorCell(point, mesh, free, admissibleLo, admissibleHi, geometry) {
    const base = cell(point, mesh), candidates = [];
    for (let di = -3; di <= 3; di++) for (let dj = -3; dj <= 3; dj++) {
      const i = base[0] + di, j = base[1] + dj;
      if (i < 0 || i >= mesh.shapeX || j < 0 || j >= mesh.shapeY
          || !free[indexOf(i, j, mesh.shapeY)]) continue;
      const dx = mesh.axisX[i] - point.x, dy = mesh.axisY[j] - point.y;
      candidates.push([dx * dx + dy * dy, i, j]);
    }
    candidates.sort(tupleCompare);
    for (const candidate of candidates) {
      const cellPoint = {x: mesh.axisX[candidate[1]], y: mesh.axisY[candidate[2]]};
      if (segmentIsSafe(
        point, cellPoint, admissibleLo, admissibleHi, geometry.centers, geometry.half
      )) return [candidate[1], candidate[2]];
    }
    return null;
  }

  function smoothRaw(raw, admissibleLo, admissibleHi, geometry) {
    const smoothed = [raw[0]];
    let anchor = 0;
    while (anchor < raw.length - 1) {
      let candidate = raw.length - 1;
      while (candidate > anchor + 1 && !segmentIsSafe(
        raw[anchor], raw[candidate], admissibleLo, admissibleHi,
        geometry.centers, geometry.half
      )) candidate--;
      if (!segmentIsSafe(
        raw[anchor], raw[candidate], admissibleLo, admissibleHi,
        geometry.centers, geometry.half
      )) return null;
      smoothed.push(raw[candidate]); anchor = candidate;
    }
    return smoothed;
  }

  function finishPlan(points, expanded, rawCount, config) {
    if (!points) return emptyPlan('smoothing_invalid', expanded);
    if (points.length > config.maxWaypoints) {
      const result = emptyPlan('waypoint_limit', expanded);
      result.rawGridNodes = rawCount; result.smoothedNodes = points.length; return result;
    }
    let length = 0;
    for (let i = 1; i < points.length; i++) {
      length += Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
    return {
      status: 'ok', valid: true, waypoints: points, expandedNodes: expanded,
      rawGridNodes: rawCount, smoothedNodes: points.length, pathLengthM: length,
    };
  }

  function normalizedConfig(overrides) {
    return Object.assign({}, CONTRACT, overrides || {});
  }

  function prepare(start, bars, arenaLo, arenaHi, support, config) {
    if (![start, arenaLo, arenaHi, support].every(finitePoint)
        || arenaHi.x <= arenaLo.x || arenaHi.y <= arenaLo.y
        || support.x < 0 || support.y < 0) return null;
    const geometry = inflatedGeometry(bars, support, config.trackingMarginM);
    if (!geometry) return null;
    const admissibleLo = {x: arenaLo.x + config.boundaryMarginM + support.x,
      y: arenaLo.y + config.boundaryMarginM + support.y};
    const admissibleHi = {x: arenaHi.x - config.boundaryMarginM - support.x,
      y: arenaHi.y - config.boundaryMarginM - support.y};
    return {geometry, admissibleLo, admissibleHi};
  }

  function plan(start, goal, bars, arenaLo, arenaHi, support, overrides) {
    const config = normalizedConfig(overrides);
    const prepared = prepare(start, bars, arenaLo, arenaHi, support, config);
    if (!prepared || !finitePoint(goal)) return emptyPlan('invalid_input');
    const {geometry, admissibleLo, admissibleHi} = prepared;
    if (!segmentIsSafe(start, start, admissibleLo, admissibleHi,
      geometry.centers, geometry.half)) return emptyPlan('unsafe_start');
    if (!segmentIsSafe(goal, goal, admissibleLo, admissibleHi,
      geometry.centers, geometry.half)) return emptyPlan('unsafe_goal');
    if (segmentIsSafe(start, goal, admissibleLo, admissibleHi,
      geometry.centers, geometry.half)) return finishPlan([start, goal], 0, 2, config);

    const mesh = grid(arenaLo, arenaHi, config.resolutionM);
    const free = occupancy(mesh, admissibleLo, admissibleHi, geometry, 0);
    const startCell = anchorCell(start, mesh, free, admissibleLo, admissibleHi, geometry);
    const goalCell = anchorCell(goal, mesh, free, admissibleLo, admissibleHi, geometry);
    if (!startCell) return emptyPlan('unsafe_start_cell');
    if (!goalCell) return emptyPlan('unsafe_goal_cell');
    const startIndex = indexOf(startCell[0], startCell[1], mesh.shapeY);
    const goalIndex = indexOf(goalCell[0], goalCell[1], mesh.shapeY);
    const size = mesh.shapeX * mesh.shapeY;
    const score = new Float64Array(size); score.fill(Infinity); score[startIndex] = 0;
    const parent = new Int32Array(size); parent.fill(-1);
    const heuristic = (i, j) => Math.hypot(i - goalCell[0], j - goalCell[1]);
    const queue = new MinHeap(tupleCompare);
    queue.push([heuristic(startCell[0], startCell[1]), 0, startCell[0], startCell[1]]);
    let expanded = 0;
    while (queue.length) {
      const item = queue.pop(), cost = item[1], i = item[2], j = item[3];
      const currentIndex = indexOf(i, j, mesh.shapeY);
      if (cost !== score[currentIndex]) continue;
      if (++expanded > config.maxExpansions) return emptyPlan('expansion_limit', expanded);
      if (currentIndex === goalIndex) break;
      for (const neighbor of NEIGHBORS) {
        const di = neighbor[0], dj = neighbor[1], ii = i + di, jj = j + dj;
        if (ii < 0 || ii >= mesh.shapeX || jj < 0 || jj >= mesh.shapeY) continue;
        const nextIndex = indexOf(ii, jj, mesh.shapeY);
        if (!free[nextIndex]) continue;
        if (di && dj && (!free[indexOf(i + di, j, mesh.shapeY)]
            || !free[indexOf(i, j + dj, mesh.shapeY)])) continue;
        const candidate = cost + neighbor[2];
        if (candidate < score[nextIndex]) {
          score[nextIndex] = candidate; parent[nextIndex] = currentIndex;
          queue.push([candidate + heuristic(ii, jj), candidate, ii, jj]);
        }
      }
    }
    if (!Number.isFinite(score[goalIndex])) return emptyPlan('no_path', expanded);
    const cells = [];
    for (let cursor = goalIndex; cursor !== startIndex; cursor = parent[cursor]) {
      if (cursor < 0) return emptyPlan('parent_chain_invalid', expanded);
      cells.push(cursor);
    }
    cells.push(startIndex); cells.reverse();
    const raw = [start];
    for (let n = 0; n < cells.length; n++) {
      const i = Math.floor(cells[n] / mesh.shapeY), j = cells[n] % mesh.shapeY;
      const point = {x: mesh.axisX[i], y: mesh.axisY[j]};
      const previous = raw[raw.length - 1];
      if (Math.hypot(point.x - previous.x, point.y - previous.y) > 1e-9) raw.push(point);
    }
    const previous = raw[raw.length - 1];
    if (Math.hypot(goal.x - previous.x, goal.y - previous.y) > 1e-9) raw.push(goal);
    return finishPlan(smoothRaw(raw, admissibleLo, admissibleHi, geometry), expanded, raw.length, config);
  }

  function planToConnectedGoal(start, bars, arenaLo, arenaHi, support, selector, overrides) {
    const config = normalizedConfig(overrides);
    if (!Number.isFinite(selector) || !(config.minGoalDistanceM > 0)) {
      return emptyPlan('invalid_input');
    }
    const prepared = prepare(start, bars, arenaLo, arenaHi, support, config);
    if (!prepared) return emptyPlan('invalid_input');
    const {geometry, admissibleLo, admissibleHi} = prepared;
    if (!segmentIsSafe(start, start, admissibleLo, admissibleHi,
      geometry.centers, geometry.half)) return emptyPlan('unsafe_start');
    const mesh = grid(arenaLo, arenaHi, config.resolutionM);
    const free = occupancy(mesh, admissibleLo, admissibleHi, geometry, 0);
    const startCell = anchorCell(start, mesh, free, admissibleLo, admissibleHi, geometry);
    if (!startCell) return emptyPlan('unsafe_start_cell');
    const startIndex = indexOf(startCell[0], startCell[1], mesh.shapeY);
    const size = mesh.shapeX * mesh.shapeY;
    const distance = new Float64Array(size); distance.fill(Infinity); distance[startIndex] = 0;
    const parent = new Int32Array(size); parent.fill(-1);
    const queue = new MinHeap(tupleCompare); queue.push([0, startCell[0], startCell[1]]);
    let expanded = 0;
    while (queue.length) {
      const item = queue.pop(), cost = item[0], i = item[1], j = item[2];
      const currentIndex = indexOf(i, j, mesh.shapeY);
      if (cost !== distance[currentIndex]) continue;
      if (++expanded > config.maxExpansions) return emptyPlan('expansion_limit', expanded);
      for (const neighbor of NEIGHBORS) {
        const di = neighbor[0], dj = neighbor[1], ii = i + di, jj = j + dj;
        if (ii < 0 || ii >= mesh.shapeX || jj < 0 || jj >= mesh.shapeY) continue;
        const nextIndex = indexOf(ii, jj, mesh.shapeY);
        if (!free[nextIndex]) continue;
        if (di && dj && (!free[indexOf(i + di, j, mesh.shapeY)]
            || !free[indexOf(i, j + dj, mesh.shapeY)])) continue;
        const candidate = cost + neighbor[2];
        if (candidate < distance[nextIndex]) {
          distance[nextIndex] = candidate; parent[nextIndex] = currentIndex;
          queue.push([candidate, ii, jj]);
        }
      }
    }
    const reachable = [];
    // np.nonzero row-major ordering: i outer, j inner.
    for (let i = 0; i < mesh.shapeX; i++) for (let j = 0; j < mesh.shapeY; j++) {
      const flat = indexOf(i, j, mesh.shapeY);
      if (Number.isFinite(distance[flat]) && Math.hypot(
        mesh.axisX[i] - start.x, mesh.axisY[j] - start.y
      ) >= config.minGoalDistanceM) reachable.push(flat);
    }
    if (!reachable.length) return emptyPlan('no_connected_goal');
    const wrapped = ((selector % 1) + 1) % 1;
    const choice = Math.min(reachable.length - 1, Math.floor(wrapped * reachable.length));
    const goalIndex = reachable[choice], reverse = [];
    for (let cursor = goalIndex; cursor !== startIndex; cursor = parent[cursor]) {
      if (cursor < 0) return emptyPlan('parent_chain_invalid', expanded);
      reverse.push(cursor);
    }
    reverse.push(startIndex); reverse.reverse();
    const raw = [start];
    for (let n = 0; n < reverse.length; n++) {
      const i = Math.floor(reverse[n] / mesh.shapeY), j = reverse[n] % mesh.shapeY;
      const point = {x: mesh.axisX[i], y: mesh.axisY[j]};
      const previous = raw[raw.length - 1];
      if (Math.hypot(point.x - previous.x, point.y - previous.y) > 1e-9) raw.push(point);
    }
    return finishPlan(smoothRaw(raw, admissibleLo, admissibleHi, geometry), expanded, raw.length, config);
  }

  return {
    CONTRACT,
    conservativeXYSupportFromBox,
    segmentIntersectsClosedAABB,
    segmentIsSafe,
    plan,
    planToConnectedGoal,
  };
});
