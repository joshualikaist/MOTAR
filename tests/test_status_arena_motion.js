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

console.log('status arena motion parity: ok');
