'use strict';

const assert = require('assert');
const Motion = require('../docs/status/arena_motion.js');

Motion.configure({arena_xy_m: 40, goal_dist_m: [6, 28], target_speed_m: [0.3, 1.5]});

// Python target_motion.py/config parity: values are lineage constants, not display tuning knobs.
assert.strictEqual(Motion.CONTRACT.boundedMaxAccel, 4.0);
assert(Math.abs(Motion.CONTRACT.boundedMaxTurnRate - 150 * Math.PI / 180) < 1e-12);
assert.strictEqual(Motion.CONTRACT.boundedLookaheadSeconds, 1.0);
assert.strictEqual(Motion.CONTRACT.boundedObstacleClearance, 0.77);

// One 10 Hz step may change velocity by at most a*dt and heading by at most omega*dt while moving.
let velocity = Motion.limitPlanarVelocity(
  {x: 0, y: 0}, {x: 1.5, y: 0}, 1.5, 0.1, 4.0, 150 * Math.PI / 180
);
assert(Math.abs(Math.hypot(velocity.x, velocity.y) - 0.4) < 1e-12);
const current = {x: 1.5, y: 0};
velocity = Motion.limitPlanarVelocity(
  current, {x: 0, y: 1.5}, 1.5, 0.1, 4.0, 150 * Math.PI / 180
);
assert(Math.hypot(velocity.x - current.x, velocity.y - current.y) <= 0.4 + 1e-12);
assert(Math.abs(Math.atan2(velocity.y, velocity.x)) <= 15 * Math.PI / 180 + 1e-12);
assert(Math.hypot(velocity.x, velocity.y) <= 1.5 + 1e-12);

// The planner uses centre-distance clearance in the bounded (non-physical) Python lineage.
const blocked = Motion.boundedTargetStep(
  {x: 10, y: 0}, {x: 0, y: 0}, {x: 1.5, y: 0}, 1.5, 0.1,
  [{x: 10.4, y: 0, w: 0.8}], 1
);
assert.strictEqual(blocked.feasible, false, 'inside-clearance starts must fail closed');
assert(Number.isFinite(blocked.firstPos.x) && Number.isFinite(blocked.firstVelocity.x));
const lookaheadTurn = Motion.boundedTargetStep(
  {x: 10, y: 0}, {x: 1.5, y: 0}, {x: 1.5, y: 0}, 1.5, 0.1,
  [{x: 11.5, y: 0, w: 0.8}], 1
);
assert.notStrictEqual(lookaheadTurn.index, 0, '1 s rollout must reject a future direct collision');
// Golden values from Python target_motion.bounded_drone_target_step for this exact float fixture.
assert(Math.abs(lookaheadTurn.firstPos.x - 10.1448888779) < 1e-7);
assert(Math.abs(lookaheadTurn.firstPos.y - 0.0388228558) < 1e-7);
assert(Math.abs(lookaheadTurn.firstVelocity.x - 1.4488886595) < 1e-7);
assert(Math.abs(lookaheadTurn.firstVelocity.y - 0.3882285357) < 1e-7);
assert.strictEqual(lookaheadTurn.feasible, true);

function deterministicEpisode() {
  const episode = Motion.createEpisode(Motion.seededRng(77), [], 1.5);
  episode.mode = 'cv';
  episode.speed = 1.5;
  episode.target = {x: 20, y: 0};
  episode.cvVelocity = {x: 1.5, y: 0};
  episode.boundedVelocity = {x: 0, y: 0};
  episode.realizedVelocity = {x: 0, y: 0};
  episode.heading = 0;
  return episode;
}

function runAtFps(fps) {
  const episode = deterministicEpisode();
  const rng = Motion.seededRng(99);
  const clock = Motion.createFixedStepClock(0.1, 0.25, 8);
  let steps = 0;
  clock.advance(0, true, function () {});
  for (let frame = 1; frame <= fps * 5; frame++) {
    clock.advance(frame / fps, true, function (dt) {
      Motion.advanceTarget(episode, dt, [], rng, 'bounded');
      steps++;
    });
  }
  // Resolve floating-point residue without adding another fixed step.
  const result = clock.advance(5 + 1e-9, true, function (dt) {
    Motion.advanceTarget(episode, dt, [], rng, 'bounded');
    steps++;
  });
  return {episode, steps, simulationTime: result.simulationTime};
}

const at30 = runAtFps(30);
const at60 = runAtFps(60);
const at144 = runAtFps(144);
for (const run of [at30, at60, at144]) {
  assert.strictEqual(run.steps, 50);
  assert(Math.abs(run.simulationTime - 5.0) < 1e-9);
}
for (const run of [at60, at144]) {
  assert(Math.abs(run.episode.target.x - at30.episode.target.x) < 1e-10);
  assert(Math.abs(run.episode.target.y - at30.episode.target.y) < 1e-10);
  assert(Math.abs(run.episode.realizedVelocity.x - at30.episode.realizedVelocity.x) < 1e-10);
}

// Render interpolation alpha is decoupled from simulation step count.
const interpolationClock = Motion.createFixedStepClock(0.1, 0.25, 8);
let interpolationSteps = 0;
interpolationClock.advance(0, true, function () {});
const half = interpolationClock.advance(0.15, true, function () { interpolationSteps++; });
assert.strictEqual(interpolationSteps, 1);
assert(Math.abs(half.alpha - 0.5) < 1e-12);

// Physical-style is explicitly only a bounded-command display filter, but it must still respect
// its declared acceleration and attitude display limits.
const physical = deterministicEpisode();
let previousVelocity = {x: 0, y: 0};
for (let i = 0; i < 40; i++) {
  Motion.advanceTarget(physical, 0.1, [], Motion.seededRng(4), 'physical-style');
  const currentVelocity = physical.physicalStyle.velocity;
  assert(Math.hypot(
    currentVelocity.x - previousVelocity.x,
    currentVelocity.y - previousVelocity.y
  ) <= 0.4 + 1e-12);
  assert(Math.abs(Math.atan(physical.physicalStyle.roll)) <= 45 * Math.PI / 180 + 1e-12);
  assert(Math.abs(Math.atan(physical.physicalStyle.pitch)) <= 45 * Math.PI / 180 + 1e-12);
  previousVelocity = {x: currentVelocity.x, y: currentVelocity.y};
}

console.log('MOTAR arena fixed-clock and target-limit contracts: PASS');
