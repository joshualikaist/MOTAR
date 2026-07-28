/*
 * Browser-side reproduction of the NavRL general-training target distribution.
 *
 * Source of truth:
 *   aerial_gym/task/navrl_task/navrl_task.py
 *     _randomize_general_drone_spawn, _sample_general_target,
 *     _sample_target_motion, _sample_waypoints, _advance_target
 *   aerial_gym/rl_training/rl_games/train_navrl_general_repr_density.sh
 *
 * Coordinates use the status arena convention: x=[0,24], y=[-12,12].
 * This module is deliberately independent of THREE/DOM so its parity contracts
 * can be tested with Node.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.NavRLArenaMotion = api;
})(typeof window !== 'undefined' ? window : this, function () {
  'use strict';

  const CONTRACT = Object.freeze({
    bounds: Object.freeze({ x0: 0, x1: 24, y0: -12, y1: 12 }),
    wallMargin: 0.5,
    spawnMargin: 1.0,
    spawnBarClearance: 0.65,
    targetBarClearance: 1.0,
    targetDistanceMin: 4.0,
    targetDistanceMax: 16.0,
    waypointReach: 0.5,
    targetSpeedMax: 1.5,
    pursuerSpeedMax: 2.5,
  });

  function seededRng(seed) {
    return function () {
      seed |= 0;
      seed = seed + 0x6D2B79F5 | 0;
      let t = Math.imul(seed ^ seed >>> 15, 1 | seed);
      t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
      return ((t ^ t >>> 14) >>> 0) / 4294967296;
    };
  }

  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function distanceToBars(x, y, bars) {
    let best = Infinity;
    for (const bar of bars || []) best = Math.min(best, Math.hypot(x - bar.x, y - bar.y));
    return best;
  }

  function samplePoint(rng, margin) {
    const b = CONTRACT.bounds;
    return {
      x: b.x0 + margin + rng() * (b.x1 - b.x0 - 2 * margin),
      y: b.y0 + margin + rng() * (b.y1 - b.y0 - 2 * margin),
    };
  }

  function sampleClearPoint(rng, bars, clearance, margin, accept) {
    let candidate = samplePoint(rng, margin);
    for (let i = 0; i < 96; i++) {
      const next = samplePoint(rng, margin);
      if (distanceToBars(next.x, next.y, bars) >= clearance && (!accept || accept(next))) {
        return next;
      }
      candidate = next;
    }
    return candidate;
  }

  function sampleWaypoint(rng) {
    return samplePoint(rng, CONTRACT.wallMargin);
  }

  function createEpisode(rng, bars, speedCeiling) {
    const drone = sampleClearPoint(
      rng, bars, CONTRACT.spawnBarClearance, CONTRACT.spawnMargin
    );
    const maxDistance = CONTRACT.targetDistanceMax;
    const target = sampleClearPoint(
      rng,
      bars,
      CONTRACT.spawnBarClearance,
      CONTRACT.spawnMargin,
      function (p) {
        const d = Math.hypot(p.x - drone.x, p.y - drone.y);
        return d >= CONTRACT.targetDistanceMin && d <= maxDistance;
      }
    );
    const mode = rng() < 0.5 ? 'cv' : 'waypoint';
    const speed = rng() * Math.max(0, speedCeiling == null ? CONTRACT.targetSpeedMax : speedCeiling);
    const angle = rng() * Math.PI * 2;
    return {
      drone: drone,
      target: target,
      mode: mode,
      speed: speed,
      cvVelocity: { x: speed * Math.cos(angle), y: speed * Math.sin(angle) },
      waypoint: sampleWaypoint(rng),
      realizedVelocity: { x: 0, y: 0 },
      age: 0,
    };
  }

  function pushOutOfBars(point, bars, clearance) {
    const b = CONTRACT.bounds;
    let x = point.x;
    let y = point.y;
    let pushed = false;
    for (let iteration = 0; iteration < 6; iteration++) {
      x = clamp(x, b.x0 + CONTRACT.wallMargin, b.x1 - CONTRACT.wallMargin);
      y = clamp(y, b.y0 + CONTRACT.wallMargin, b.y1 - CONTRACT.wallMargin);
      let sumX = 0, sumY = 0, maxPenetration = 0, nearest = null, nearestD = Infinity;
      for (const bar of bars || []) {
        const dx = x - bar.x, dy = y - bar.y;
        const d = Math.hypot(dx, dy);
        if (d < nearestD) { nearestD = d; nearest = { dx: dx, dy: dy }; }
        if (d < clearance) {
          const denom = Math.max(d, 1e-6);
          sumX += dx / denom;
          sumY += dy / denom;
          maxPenetration = Math.max(maxPenetration, clearance - d);
        }
      }
      if (maxPenetration <= 0) break;
      pushed = true;
      let norm = Math.hypot(sumX, sumY);
      if (norm <= 1e-6 && nearest) {
        sumX = nearest.dx;
        sumY = nearest.dy;
        norm = Math.max(Math.hypot(sumX, sumY), 1e-6);
      }
      x += sumX / norm * (maxPenetration + 1e-3);
      y += sumY / norm * (maxPenetration + 1e-3);
    }
    return { x: x, y: y, pushed: pushed };
  }

  function advanceTarget(episode, dt, bars, rng) {
    if (!episode || dt <= 0 || episode.speed <= 1e-6) return episode;
    const b = CONTRACT.bounds;
    const loX = b.x0 + CONTRACT.wallMargin, hiX = b.x1 - CONTRACT.wallMargin;
    const loY = b.y0 + CONTRACT.wallMargin, hiY = b.y1 - CONTRACT.wallMargin;
    const oldX = episode.target.x, oldY = episode.target.y;
    let x = oldX, y = oldY;

    if (episode.mode === 'cv') {
      x += episode.cvVelocity.x * dt;
      y += episode.cvVelocity.y * dt;
      if (x < loX) { x = 2 * loX - x; episode.cvVelocity.x *= -1; }
      if (x > hiX) { x = 2 * hiX - x; episode.cvVelocity.x *= -1; }
      if (y < loY) { y = 2 * loY - y; episode.cvVelocity.y *= -1; }
      if (y > hiY) { y = 2 * hiY - y; episode.cvVelocity.y *= -1; }
    } else {
      const wx = episode.waypoint.x - oldX, wy = episode.waypoint.y - oldY;
      const d = Math.max(Math.hypot(wx, wy), 1e-6);
      const step = Math.min(episode.speed * dt, d);
      x += wx / d * step;
      y += wy / d * step;
      if (Math.hypot(episode.waypoint.x - x, episode.waypoint.y - y) < CONTRACT.waypointReach) {
        episode.waypoint = sampleWaypoint(rng);
      }
    }

    const clear = pushOutOfBars({ x: x, y: y }, bars, CONTRACT.targetBarClearance);
    episode.target.x = clamp(clear.x, loX, hiX);
    episode.target.y = clamp(clear.y, loY, hiY);
    episode.realizedVelocity.x = (episode.target.x - oldX) / dt;
    episode.realizedVelocity.y = (episode.target.y - oldY) / dt;
    if (clear.pushed && episode.mode === 'waypoint') episode.waypoint = sampleWaypoint(rng);
    episode.age += dt;
    return episode;
  }

  return {
    CONTRACT: CONTRACT,
    seededRng: seededRng,
    distanceToBars: distanceToBars,
    sampleWaypoint: sampleWaypoint,
    createEpisode: createEpisode,
    pushOutOfBars: pushOutOfBars,
    advanceTarget: advanceTarget,
  };
});
