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

const status = JSON.parse(fs.readFileSync(path.join(repo, 'docs/status/status.json'), 'utf8'));
assert(status.research_update);
assert.strictEqual(status.research_update.legacy_eval.length, 5);
assert.strictEqual(status.research_update.bounded_pilot.run,
  'ppo_260729_2225_navrl_corrected-squashed-fresh-pilot-s1');
assert.strictEqual(status.research_update.bounded_pilot.raw_oob_y, 0);
assert(status.research_update.milestones.some(m => m.label === 'SENSOR GEOMETRY'));

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
