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
    pursuerRadius: 0.25,
    pursuerLookaheadSeconds: 0.9,
  });
  const TURN_ANGLES = Object.freeze(
    [0, 30, -30, 60, -60, 90, -90, 120, -120, 180].map(
      function (degrees) { return degrees * Math.PI / 180; }
    )
  );

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

  function pursuerClearance(x, y, bars) {
    let best = Infinity;
    for (const bar of bars || []) {
      const radius = CONTRACT.pursuerRadius + 0.5 * (bar.w == null ? 0.6 : bar.w);
      best = Math.min(best, Math.hypot(x - bar.x, y - bar.y) - radius);
    }
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
      CONTRACT.targetBarClearance,
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
      avoidSign: rng() < 0.5 ? -1 : 1,
      realizedVelocity: { x: 0, y: 0 },
      age: 0,
    };
  }

  function pushOutOfBars(point, bars, clearance) {
    const b = CONTRACT.bounds;
    let x = point.x;
    let y = point.y;
    let pushed = false;
    let normalX = 0, normalY = 0;
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
      normalX = sumX / norm;
      normalY = sumY / norm;
      x += normalX * (maxPenetration + 1e-3);
      y += normalY * (maxPenetration + 1e-3);
    }
    return {
      x: x, y: y, pushed: pushed, normalX: normalX, normalY: normalY
    };
  }

  function steerTargetStep(oldX, oldY, desiredVX, desiredVY, speed, dt, bars, avoidSign) {
    const b = CONTRACT.bounds;
    const loX = b.x0 + CONTRACT.wallMargin, hiX = b.x1 - CONTRACT.wallMargin;
    const loY = b.y0 + CONTRACT.wallMargin, hiY = b.y1 - CONTRACT.wallMargin;
    const norm = Math.max(Math.hypot(desiredVX, desiredVY), 1e-6);
    const bx = desiredVX / norm, by = desiredVY / norm;
    let best = null;
    TURN_ANGLES.forEach(function (angle, index) {
      const c = Math.cos(angle), s = Math.sin(angle);
      const dx = bx * c - by * s, dy = bx * s + by * c;
      const x = oldX + dx * speed * dt, y = oldY + dy * speed * dt;
      const inside = x >= loX && x <= hiX && y >= loY && y <= hiY;
      const minBar = distanceToBars(x, y, bars);
      const clear = inside && minBar >= CONTRACT.targetBarClearance + 1e-4;
      const tie = (avoidSign || 1) * Math.sign(angle) * 1e-3;
      const score = clear
        ? 1000 - Math.abs(angle) + tie
        : (inside ? 100 : 0) + Math.min(minBar, 10) - 0.01 * Math.abs(angle) + tie;
      if (!best || score > best.score) {
        best = {
          x: x, y: y, vx: dx * speed, vy: dy * speed,
          clear: clear, steered: index !== 0, score: score,
        };
      }
    });
    return best;
  }

  /*
   * Collision-aware illustrative pursuer for the browser scene.
   *
   * This is intentionally not presented as the learned PyTorch policy.  The old
   * attractive-plus-repulsive potential field had local minima in dense layouts:
   * symmetric bar forces could exactly cancel the target direction.  Here we
   * score persistent left/right heading candidates over a short swept path.
   */
  function steerPursuerStep(oldX, oldY, targetX, targetY, speed, dt, bars, oldHeading, avoidSign) {
    const b = CONTRACT.bounds;
    const loX = b.x0 + CONTRACT.wallMargin, hiX = b.x1 - CONTRACT.wallMargin;
    const loY = b.y0 + CONTRACT.wallMargin, hiY = b.y1 - CONTRACT.wallMargin;
    const tx = targetX - oldX, ty = targetY - oldY;
    const targetDistance = Math.hypot(tx, ty);
    if (dt <= 0 || speed <= 1e-6 || targetDistance <= 1e-6) {
      return { x: oldX, y: oldY, vx: 0, vy: 0, heading: oldHeading || 0, blocked: false };
    }

    const directHeading = Math.atan2(ty, tx);
    const offsets = [0, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90, 120, -120, 180];
    const stepDistance = Math.min(speed * dt, targetDistance);
    const lookDistance = Math.min(
      speed * CONTRACT.pursuerLookaheadSeconds,
      Math.max(stepDistance, targetDistance)
    );
    const samples = Math.max(2, Math.ceil(lookDistance / 0.18));
    let best = null;

    offsets.forEach(function (degrees) {
      const angle = directHeading + degrees * Math.PI / 180;
      const ux = Math.cos(angle), uy = Math.sin(angle);
      let minClearance = Infinity;
      let immediateClear = true;
      let pathClear = true;
      for (let i = 1; i <= samples; i++) {
        const distance = lookDistance * i / samples;
        const x = oldX + ux * distance, y = oldY + uy * distance;
        const inside = x >= loX && x <= hiX && y >= loY && y <= hiY;
        const clearance = pursuerClearance(x, y, bars);
        minClearance = Math.min(minClearance, clearance);
        if (!inside || clearance < 0) {
          pathClear = false;
          if (distance <= stepDistance + 1e-6) immediateClear = false;
        }
      }
      if (!immediateClear) return;

      const endX = oldX + ux * lookDistance, endY = oldY + uy * lookDistance;
      const progress = targetDistance - Math.hypot(targetX - endX, targetY - endY);
      const turn = Math.abs(Math.atan2(
        Math.sin(angle - (oldHeading == null ? directHeading : oldHeading)),
        Math.cos(angle - (oldHeading == null ? directHeading : oldHeading))
      ));
      const sideBias = (avoidSign || 1) * Math.sign(degrees) * 0.03;
      const score = (pathClear ? 1000 : 0)
        + progress * 8
        + Math.min(minClearance, 2) * 0.8
        - turn * 0.35
        - Math.abs(degrees) * 0.002
        + sideBias;
      if (!best || score > best.score) {
        best = { angle: angle, ux: ux, uy: uy, pathClear: pathClear, score: score };
      }
    });

    if (!best) {
      return {
        x: oldX, y: oldY, vx: 0, vy: 0,
        heading: oldHeading == null ? directHeading : oldHeading, blocked: true
      };
    }
    return {
      x: clamp(oldX + best.ux * stepDistance, loX, hiX),
      y: clamp(oldY + best.uy * stepDistance, loY, hiY),
      vx: best.ux * speed,
      vy: best.uy * speed,
      heading: best.angle,
      blocked: !best.pathClear,
    };
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

    // Match target_motion.steer_target_step: keep the direct full-speed heading when clear,
    // otherwise take the smallest symmetric turn that clears the bars and arena wall.
    const steered = steerTargetStep(
      oldX, oldY, (x - oldX) / dt, (y - oldY) / dt,
      episode.speed, dt, bars, episode.avoidSign
    );
    x = steered.x;
    y = steered.y;
    if (episode.mode === 'cv') {
      episode.cvVelocity.x = steered.vx;
      episode.cvVelocity.y = steered.vy;
    }

    const clear = pushOutOfBars({ x: x, y: y }, bars, CONTRACT.targetBarClearance);
    episode.target.x = clamp(clear.x, loX, hiX);
    episode.target.y = clamp(clear.y, loY, hiY);
    episode.realizedVelocity.x = (episode.target.x - oldX) / dt;
    episode.realizedVelocity.y = (episode.target.y - oldY) / dt;
    if (clear.pushed && episode.mode === 'waypoint') episode.waypoint = sampleWaypoint(rng);
    if (clear.pushed && episode.mode === 'cv') {
      const into = episode.cvVelocity.x * clear.normalX
        + episode.cvVelocity.y * clear.normalY;
      if (into < 0) {
        const vx = episode.cvVelocity.x - 2 * into * clear.normalX;
        const vy = episode.cvVelocity.y - 2 * into * clear.normalY;
        const jitter = (rng() - 0.5) * Math.PI / 9;
        const c = Math.cos(jitter), s = Math.sin(jitter);
        episode.cvVelocity.x = vx * c - vy * s;
        episode.cvVelocity.y = vx * s + vy * c;
      }
    }
    episode.age += dt;
    return episode;
  }

  return {
    CONTRACT: CONTRACT,
    seededRng: seededRng,
    distanceToBars: distanceToBars,
    pursuerClearance: pursuerClearance,
    sampleWaypoint: sampleWaypoint,
    createEpisode: createEpisode,
    pushOutOfBars: pushOutOfBars,
    steerTargetStep: steerTargetStep,
    steerPursuerStep: steerPursuerStep,
    advanceTarget: advanceTarget,
  };
});
