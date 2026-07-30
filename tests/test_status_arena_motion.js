'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const M = require('../docs/status/arena_motion.js');

function close(a, b, eps) {
  assert(Math.abs(a - b) <= eps, `${a} != ${b} (eps=${eps})`);
}

// Contract values mirror the active general-representation recipe.
assert.deepStrictEqual(M.CONTRACT.bounds, { x0: 0, x1: 24, y0: -12, y1: 12 });
assert.strictEqual(M.CONTRACT.targetDistanceMin, 4);
assert.strictEqual(M.CONTRACT.targetDistanceMax, 16);
assert.strictEqual(M.CONTRACT.targetBarClearance, 1);
assert.strictEqual(M.CONTRACT.waypointReach, 0.5);
assert.strictEqual(M.CONTRACT.targetSpeedMax, 1.5);
assert.strictEqual(M.CONTRACT.pursuerSpeedMax, 2.5);

// Fail loudly if the training recipe changes without updating the status simulator.
const repo = path.resolve(__dirname, '..');
const launcher = fs.readFileSync(path.join(
  repo, 'aerial_gym/rl_training/rl_games/train_navrl_general_repr_density.sh'
), 'utf8');
const taskCfg = fs.readFileSync(path.join(
  repo, 'aerial_gym/config/task_config/navrl_task_config.py'
), 'utf8');
const envCfg = fs.readFileSync(path.join(
  repo, 'aerial_gym/config/env_config/navrl_bars_env.py'
), 'utf8');
const html = fs.readFileSync(path.join(repo, 'docs/status/index.html'), 'utf8');
const suppress15Launcher = fs.readFileSync(path.join(
  repo, 'aerial_gym/rl_training/rl_games/train_navrl_corrected_squashed_density_suppress15.sh'
), 'utf8');
const clusterSectorLauncher = fs.readFileSync(path.join(
  repo, 'aerial_gym/rl_training/rl_games/train_navrl_corrected_squashed_density_cluster_sector.sh'
), 'utf8');
assert(launcher.includes('export NAVRL_GENERAL_TRAIN=1'));
assert(launcher.includes('export NAVRL_TARGET_SPEED_MIN=0.0'));
assert(launcher.includes('export NAVRL_TARGET_SPEED_FINAL=1.5'));
assert(launcher.includes('export NAVRL_TARGET_PATTERN=mixed'));
assert(launcher.includes('export NAVRL_OBSTACLE_FOV_DEG=240'));
assert(launcher.includes('export NAVRL_LIDAR_HBEAMS=72'));
assert(launcher.includes('export NAVRL_LIDAR_VBEAMS=4'));
assert(launcher.includes('export NAVRL_LIDAR_RANGE=12'));
assert(launcher.includes('export NAVRL_K_FINAL=16'));
assert(launcher.includes('export NAVRL_MAX_VELOCITY=2.5'));
assert(launcher.includes('NAVRL_OBSTACLE_SUPPRESS_DEG="${NAVRL_OBSTACLE_SUPPRESS_DEG:-10}"'));
assert(suppress15Launcher.includes('export NAVRL_OBSTACLE_SUPPRESS_DEG=15'));
assert(suppress15Launcher.includes('export NAVRL_ALLOW_SUPPRESS_WARMSTART=1'));
assert(clusterSectorLauncher.includes('export NAVRL_OBSTACLE_SELECTOR=cluster_sector'));
assert(clusterSectorLauncher.includes('NAVRL_OBSTACLE_CLUSTER_GAP_M'));
assert(clusterSectorLauncher.includes('NAVRL_OBSTACLE_SECTORS'));
assert(taskCfg.includes('waypoint_reach_m = 0.5'));
assert(taskCfg.includes('goal_min_bar_clearance = 1.0'));
assert(taskCfg.includes('detector_hfov_deg = 87.0'));
assert(taskCfg.includes('detector_max_range = 20.0'));
assert(envCfg.includes('upper_bound_min = [24.0, 24.0, 3.0]'));
const lidarCfg = fs.readFileSync(path.join(
  repo, 'aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py'
), 'utf8');
assert(lidarCfg.includes('vertical_fov_deg_min = -10.0'));
assert(lidarCfg.includes('vertical_fov_deg_max = 20.0'));
assert(html.includes('camera 87° @20 m · obstacle tokens 240°'));
assert(!html.includes('detector 240°'));
assert(html.indexOf('arena_motion.js') < html.indexOf('arena.js'));
assert(html.includes('id="panel-update"'));
assert(html.includes('tanh-squashed Gaussian · bounded'));
assert(html.includes('fixed σ [.35,.35,.05,.08]'));
assert(html.includes('cluster 0.45 m · 8 angular sectors'));

const status = JSON.parse(fs.readFileSync(path.join(repo, 'docs/status/status.json'), 'utf8'));
assert(status.research_update);
assert(status.research_update.active_experiment.run.endsWith(
  'navrl_corrected-squashed-density-cluster-sector-s1'
));
assert.strictEqual(status.research_update.active_experiment.selector, 'cluster_sector');
assert.strictEqual(status.research_update.active_experiment.cluster_gap_m, 0.45);
assert.strictEqual(status.research_update.active_experiment.sectors, 8);
assert.strictEqual(status.research_update.comparison.length, 5);
assert(status.research_update.milestones.some(m => m.label === 'SENSOR GEOMETRY'));
assert(status.research_update.milestones.some(m => m.label === 'DE-DUPLICATION'));
assert(status.density_curves.corrected_chirality_density_curve);

// Mixed must really contain both 2-D CV and waypoint episodes, with non-axis-only CV headings.
const rng = M.seededRng(20260728);
let cv = 0, waypoint = 0, cvX = 0, cvY = 0;
for (let i = 0; i < 400; i++) {
  const ep = M.createEpisode(rng, [], 1.5);
  const d = Math.hypot(ep.target.x - ep.drone.x, ep.target.y - ep.drone.y);
  assert(d >= 4 && d <= 16);
  assert(ep.speed >= 0 && ep.speed <= 1.5);
  if (ep.mode === 'cv') {
    cv++;
    if (Math.abs(ep.cvVelocity.x) > 0.1) cvX++;
    if (Math.abs(ep.cvVelocity.y) > 0.1) cvY++;
  } else waypoint++;
}
assert(cv > 150 && waypoint > 150);
assert(cvX > 100 && cvY > 100);

// Unobstructed CV integrates at the sampled physical speed and reflects on both axes.
const cvEp = {
  target: { x: 23.45, y: 11.45 }, mode: 'cv', speed: 1,
  cvVelocity: { x: 0.8, y: 0.6 }, waypoint: { x: 0, y: 0 },
  realizedVelocity: { x: 0, y: 0 }, age: 0,
};
M.advanceTarget(cvEp, 0.2, [], rng);
assert(cvEp.target.x <= 23.5 && cvEp.target.y <= 11.5);
assert(cvEp.cvVelocity.x < 0 && cvEp.cvVelocity.y < 0);

const freeEp = {
  target: { x: 10, y: 0 }, mode: 'cv', speed: 1.2,
  cvVelocity: { x: 0.72, y: 0.96 }, waypoint: { x: 0, y: 0 },
  realizedVelocity: { x: 0, y: 0 }, age: 0,
};
M.advanceTarget(freeEp, 0.1, [], rng);
close(Math.hypot(freeEp.realizedVelocity.x, freeEp.realizedVelocity.y), 1.2, 1e-10);
assert(freeEp.target.x !== 10 && freeEp.target.y !== 0);

// Composite push-out obeys the task's center-distance clearance.
const pushed = M.pushOutOfBars({ x: 10, y: 0 }, [{ x: 10, y: 0.2 }], 1);
assert(M.distanceToBars(pushed.x, pushed.y, [{ x: 10, y: 0.2 }]) >= 0.999);

// A blocked target takes a symmetric local turn without losing commanded speed.
const steered = M.steerTargetStep(
  10, 0, 1, 0, 1, 0.1, [{ x: 11.05, y: 0 }], 1
);
assert(steered.clear && steered.steered);
close(Math.hypot(steered.x - 10, steered.y), 0.1, 1e-10);
close(Math.hypot(steered.vx, steered.vy), 1, 1e-10);
assert(M.distanceToBars(steered.x, steered.y, [{ x: 11.05, y: 0 }]) >= 1);

// ---- freeze / heading-churn regression block (2026-07-30 arena-motion audit) ----

// Mirror of arena.js placeBars so density-dependent motion can be tested headlessly.
function barsAt(seed, n) {
  const r = M.seededRng(seed), pts = [];
  let spacing = 1.5, fails = 0, guard = 0;
  while (pts.length < n && guard < n * 400) {
    guard++;
    const x = 1.2 + r() * 21.6, y = -10.8 + r() * 21.6;
    let ok = true;
    for (const p of pts) {
      if ((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) < spacing * spacing) { ok = false; break; }
    }
    if (ok) { pts.push({ x: x, y: y, w: 0.4 + r() * 0.4 }); fails = 0; }
    else { fails++; if (fails >= 128) { spacing *= 0.8; fails = 0; } }
  }
  return pts;
}
function wrapAngle(a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

// T1 — episode.age is wall-clock even for a static target: arena.js's 30 s watchdog
// must not be disable-able by dragging the speed slider to 0.
const ageEp = M.createEpisode(M.seededRng(11), [], 1.5);
ageEp.speed = 0; ageEp.age = 0;
for (let i = 0; i < 100; i++) M.advanceTarget(ageEp, 0.1, [], M.seededRng(12));
close(ageEp.age, 10, 1e-9);

// T2 — rendered target yaw is temporally coherent at production density. Guards the
// heading-continuity window: pre-fix this measures 75.8°/step mean and 41% reversals.
const churnRng = M.seededRng(4242);
let churnFrames = 0, churnSum = 0, churnBig = 0;
for (let e = 0; e < 40; e++) {
  const layout = barsAt(7000 + e, 110);
  const ep = M.createEpisode(churnRng, layout, 1.5);
  if (ep.speed < 0.75) ep.speed = 0.75 + churnRng() * 0.75; // skip the by-design static half
  ep.cvVelocity.x = ep.speed * Math.cos(churnRng() * 6.283);
  ep.cvVelocity.y = ep.speed * Math.sin(churnRng() * 6.283);
  ep.heading = Math.atan2(ep.cvVelocity.y, ep.cvVelocity.x);
  let prev = null;
  for (let i = 0; i < 200; i++) {
    M.advanceTarget(ep, 0.1, layout, churnRng);
    const v = ep.realizedVelocity;
    if (Math.hypot(v.x, v.y) <= 1e-6) continue;
    const h = Math.atan2(v.y, v.x);
    if (prev != null) {
      const d = Math.abs(wrapAngle(h - prev));
      churnFrames++; churnSum += d;
      if (d > Math.PI / 2) churnBig++;
    }
    prev = h;
  }
}
const meanTurnDeg = (churnSum / churnFrames) * 180 / Math.PI;
assert(meanTurnDeg < 30, `target yaw churn too high: ${meanTurnDeg.toFixed(1)} deg/step`);
assert(churnBig / churnFrames < 0.12,
  `target reverses too often: ${(100 * churnBig / churnFrames).toFixed(1)}% of steps > 90 deg`);

// T3 — the continuity window is a preference, never a veto: with the previous heading
// pointing +x, this wall blocks every candidate within ±120° (bar centers sit in the
// 0.88..1.12 m annulus where they discriminate between 0.12 m candidate steps) and leaves
// ONLY the 180° reversal clear. The window (90°) excludes it, so the fallback chain must
// still take it rather than force a blocked step.
const wall = [{ x: 11.0, y: 0, w: 0.6 }, { x: 10, y: 1.05, w: 0.6 }, { x: 10, y: -1.05, w: 0.6 }];
const esc = M.steerTargetStep(10, 0, 1, 0, 1.2, 0.1, wall, 1, 0);
assert(M.distanceToBars(esc.x, esc.y, wall) >= 1,
  'continuity window must never force a step inside bar clearance');
close(Math.hypot(esc.vx, esc.vy), 1.2, 1e-10);

// T4 — steerTargetStep with no previous heading is byte-identical to the legacy law,
// so the 9th argument is strictly opt-in.
const legacy = M.steerTargetStep(10, 0, 1, 0, 1, 0.1, [{ x: 11.05, y: 0 }], 1);
const withNull = M.steerTargetStep(10, 0, 1, 0, 1, 0.1, [{ x: 11.05, y: 0 }], 1, null);
close(legacy.x, withNull.x, 0); close(legacy.y, withNull.y, 0);
close(legacy.vx, withNull.vx, 0); close(legacy.vy, withNull.vy, 0);

// T5 — the pursuer still crosses a dense 85-bar field end to end (anti-regression lock;
// the 4-bar trap below does not exercise realistic density).
const crossBars = barsAt(7001, 85).filter(b =>
  Math.hypot(b.x - 2, b.y) > 1.3 && Math.hypot(b.x - 22, b.y) > 1.3);
let px = 2, py = 0, ph = 0, closestCross = Infinity;
for (let i = 0; i < 1800; i++) {
  const n = M.steerPursuerStep(px, py, 22, 0, 2.5, 1 / 60, crossBars, ph, 1);
  if (n.hit) break;
  px = n.x; py = n.y; ph = n.heading;
  closestCross = Math.min(closestCross, Math.hypot(22 - px, py));
  assert(M.pursuerClearance(px, py, crossBars) >= -1e-9);
}
assert(closestCross < 0.6,
  `pursuer failed to cross an 85-bar field: closest ${closestCross.toFixed(2)} m`);

// The browser pursuer must escape the symmetric potential-field trap that used
// to make it stand still between dense bars.
const trapBars = [
  { x: 11, y: -0.45, w: 0.7 },
  { x: 11, y: 0.45, w: 0.7 },
  { x: 12.2, y: -1.25, w: 0.7 },
  { x: 12.2, y: 1.25, w: 0.7 },
];
let pursuer = { x: 10, y: 0, heading: 0 };
let moved = 0;
for (let i = 0; i < 100; i++) {
  const next = M.steerPursuerStep(
    pursuer.x, pursuer.y, 16, 0, 2.5, 0.05, trapBars, pursuer.heading, 1
  );
  moved += Math.hypot(next.x - pursuer.x, next.y - pursuer.y);
  pursuer = { x: next.x, y: next.y, heading: next.heading };
  assert(M.pursuerClearance(pursuer.x, pursuer.y, trapBars) >= -1e-9);
}
assert(moved > 5.5, `dense-trap movement too small: ${moved}`);
assert(Math.hypot(16 - pursuer.x, pursuer.y) < 1);

// Rendered bars are boxes. A circular check of radius w/2 would miss the corners;
// the pursuer disk must stay outside the AABB (including corners).
const cornerBar = [{ x: 12, y: 0, w: 0.8 }];
assert(M.pursuerClearance(12 + 0.4 * Math.SQRT2 - 0.01, 0.4 * Math.SQRT2 - 0.01, cornerBar) < 0);
const fromOutside = M.steerPursuerStep(
  12 + 0.9, 0.9, 12, 0, 2.5, 0.05, cornerBar, Math.PI, 1
);
assert(M.pursuerClearance(fromOutside.x, fromOutside.y, cornerBar) >= -1e-9);
assert(!fromOutside.hit || (fromOutside.x === 12 + 0.9 && fromOutside.y === 0.9));

// Sweep along the box diagonal must not tunnel into the pillar.
let diag = { x: 12 + 1.2, y: 1.2, heading: -3 * Math.PI / 4 };
for (let i = 0; i < 40; i++) {
  const next = M.steerPursuerStep(
    diag.x, diag.y, 12, 0, 2.5, 0.05, cornerBar, diag.heading, 1
  );
  if (next.hit) break;
  diag = { x: next.x, y: next.y, heading: next.heading };
  assert(M.pursuerClearance(diag.x, diag.y, cornerBar) >= -1e-9,
    `corner clip at step ${i}: (${diag.x}, ${diag.y})`);
}

console.log('status arena motion parity: ok');
