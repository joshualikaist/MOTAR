/*
 * Browser-side reproduction of the NavRL general-training target distribution.
 *
 * Source of truth:
 *   aerial_gym/task/navrl_task/navrl_task.py
 *     _randomize_general_drone_spawn, _sample_general_target,
 *     _sample_target_motion, _sample_waypoints, _advance_target
 *   aerial_gym/rl_training/rl_games/train_navrl_general_repr_density.sh
 *
 * Target steering parity: both this browser model and target_motion.steer_target_step use
 * the same 90° heading-continuity preference with a large-turn escape fallback.
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

  function barHalfExtent(bar) {
    return 0.5 * (bar && bar.w != null ? bar.w : 0.6);
  }

  function distanceToBars(x, y, bars) {
    let best = Infinity;
    for (const bar of bars || []) best = Math.min(best, Math.hypot(x - bar.x, y - bar.y));
    return best;
  }

  /*
   * Bars are rendered as axis-aligned boxes of side w. A circle of radius w/2 does
   * NOT cover the box corners (half-diagonal is w/2 * √2), which made the browser
   * pursuer clip through pillars even while "clearance" stayed non-negative.
   * Signed distance from a point to the box, minus the pursuer radius.
   */
  function pursuerClearance(x, y, bars) {
    let best = Infinity;
    const r = CONTRACT.pursuerRadius;
    for (const bar of bars || []) {
      const half = barHalfExtent(bar);
      const ox = Math.abs(x - bar.x) - half;
      const oy = Math.abs(y - bar.y) - half;
      let surface;
      if (ox > 0 && oy > 0) surface = Math.hypot(ox, oy);
      else if (ox > 0) surface = ox;
      else if (oy > 0) surface = oy;
      else surface = -Math.min(-ox, -oy); // inside AABB: negative depth to nearest face
      best = Math.min(best, surface - r);
    }
    return best;
  }

  function segmentPursuerClearance(x0, y0, x1, y1, bars) {
    const length = Math.hypot(x1 - x0, y1 - y0);
    const steps = Math.max(2, Math.ceil(length / 0.08));
    let best = Infinity;
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      best = Math.min(
        best,
        pursuerClearance(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, bars)
      );
    }
    return best;
  }

  function pushPursuerOut(point, bars) {
    const b = CONTRACT.bounds;
    let x = point.x;
    let y = point.y;
    let pushed = false;
    const r = CONTRACT.pursuerRadius;
    for (let iteration = 0; iteration < 8; iteration++) {
      x = clamp(x, b.x0 + CONTRACT.wallMargin, b.x1 - CONTRACT.wallMargin);
      y = clamp(y, b.y0 + CONTRACT.wallMargin, b.y1 - CONTRACT.wallMargin);
      let worst = null;
      for (const bar of bars || []) {
        const half = barHalfExtent(bar);
        const cx = clamp(x, bar.x - half, bar.x + half);
        const cy = clamp(y, bar.y - half, bar.y + half);
        let nx = x - cx, ny = y - cy;
        let d = Math.hypot(nx, ny);
        let pen;
        if (d < 1e-8) {
          // Centered inside the box: escape through the nearest face.
          const left = x - (bar.x - half);
          const right = (bar.x + half) - x;
          const bottom = y - (bar.y - half);
          const top = (bar.y + half) - y;
          const m = Math.min(left, right, bottom, top);
          if (m === left) { nx = -1; ny = 0; pen = left + r; }
          else if (m === right) { nx = 1; ny = 0; pen = right + r; }
          else if (m === bottom) { nx = 0; ny = -1; pen = bottom + r; }
          else { nx = 0; ny = 1; pen = top + r; }
        } else {
          pen = r - d;
          nx /= d;
          ny /= d;
        }
        if (pen > 1e-6 && (!worst || pen > worst.pen)) {
          worst = { nx: nx, ny: ny, pen: pen };
        }
      }
      if (!worst) break;
      pushed = true;
      x += worst.nx * (worst.pen + 1e-3);
      y += worst.ny * (worst.pen + 1e-3);
    }
    return {
      x: clamp(x, b.x0 + CONTRACT.wallMargin, b.x1 - CONTRACT.wallMargin),
      y: clamp(y, b.y0 + CONTRACT.wallMargin, b.y1 - CONTRACT.wallMargin),
      pushed: pushed,
      clear: pursuerClearance(x, y, bars) >= -1e-6,
    };
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
      rng,
      bars,
      CONTRACT.spawnBarClearance,
      CONTRACT.spawnMargin,
      function (p) {
        // Also keep the pursuer disk outside the rendered box (not just the center-distance rule).
        return pursuerClearance(p.x, p.y, bars) >= 0.05;
      }
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
      heading: angle, // last FLOWN heading, persisted for ALL patterns (see steerTargetStep)
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

  /*
   * Heading-continuity window for the target. Without it the argmax below is memoryless:
   * in waypoint mode the desired bearing is re-derived from the same static waypoint every
   * frame, so a blocking bar makes the choice flip between "aim at waypoint" and a ±120-180°
   * escape turn — a period-2 oscillation that renders as frantic spinning (measured 55°/step
   * mean heading change at 85 bars) or, at high density, as standing still at full commanded
   * speed. The window is a PREFERENCE among clear candidates, never a veto: the
   * inWindow || anyClear || trapped fallback chain must stay intact so a large escape turn
   * (and cv wall reflections) are still taken when nothing inside the window is clear.
   */
  const CONTINUITY_WINDOW = 90 * Math.PI / 180;

  function steerTargetStep(oldX, oldY, desiredVX, desiredVY, speed, dt, bars, avoidSign, prevHeading) {
    const b = CONTRACT.bounds;
    const loX = b.x0 + CONTRACT.wallMargin, hiX = b.x1 - CONTRACT.wallMargin;
    const loY = b.y0 + CONTRACT.wallMargin, hiY = b.y1 - CONTRACT.wallMargin;
    const norm = Math.max(Math.hypot(desiredVX, desiredVY), 1e-6);
    const bx = desiredVX / norm, by = desiredVY / norm;
    let inWindow = null, anyClear = null, trapped = null;
    TURN_ANGLES.forEach(function (angle, index) {
      const c = Math.cos(angle), s = Math.sin(angle);
      const dx = bx * c - by * s, dy = bx * s + by * c;
      const x = oldX + dx * speed * dt, y = oldY + dy * speed * dt;
      const inside = x >= loX && x <= hiX && y >= loY && y <= hiY;
      const minBar = distanceToBars(x, y, bars);
      const clear = inside && minBar >= CONTRACT.targetBarClearance + 1e-4;
      const tie = (avoidSign || 1) * Math.sign(angle) * 1e-3;
      const heading = Math.atan2(dy, dx);
      const candidate = {
        x: x, y: y, vx: dx * speed, vy: dy * speed,
        clear: clear, steered: index !== 0, heading: heading,
        score: clear
          ? 1000 - Math.abs(angle) + tie
          : (inside ? 100 : 0) + Math.min(minBar, 10) - 0.01 * Math.abs(angle) + tie,
      };
      if (!clear) {
        if (!trapped || candidate.score > trapped.score) trapped = candidate;
        return;
      }
      if (!anyClear || candidate.score > anyClear.score) anyClear = candidate;
      if (prevHeading == null) return;
      const turn = Math.abs(Math.atan2(
        Math.sin(heading - prevHeading), Math.cos(heading - prevHeading)
      ));
      if (turn <= CONTINUITY_WINDOW + 1e-9
          && (!inWindow || candidate.score > inWindow.score)) inWindow = candidate;
    });
    // Clear ALWAYS beats blocked (min clear score ≈ 996.86 > max trapped score ≈ 110.001);
    // with prevHeading == null, inWindow stays null and the result is exactly the legacy
    // single-pass argmax, so the 9th argument is strictly opt-in.
    return inWindow || anyClear || trapped;
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
      return { x: oldX, y: oldY, vx: 0, vy: 0, heading: oldHeading || 0, blocked: false, hit: false };
    }

    // Already inside a pillar: freeze and signal a crash so the demo can reset.
    if (pursuerClearance(oldX, oldY, bars) < -1e-4) {
      return {
        x: oldX, y: oldY, vx: 0, vy: 0,
        heading: oldHeading == null ? Math.atan2(ty, tx) : oldHeading,
        blocked: true, hit: true,
      };
    }

    const directHeading = Math.atan2(ty, tx);
    const offsets = [0, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90, 120, -120, 150, -150, 180];
    const stepDistance = Math.min(speed * dt, targetDistance);
    const lookDistance = Math.min(
      speed * CONTRACT.pursuerLookaheadSeconds,
      Math.max(stepDistance, targetDistance)
    );
    const samples = Math.max(4, Math.ceil(lookDistance / 0.1));
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
      const stepX = oldX + ux * stepDistance, stepY = oldY + uy * stepDistance;
      if (segmentPursuerClearance(oldX, oldY, stepX, stepY, bars) < 0) immediateClear = false;
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
        heading: oldHeading == null ? directHeading : oldHeading,
        blocked: true, hit: false,
      };
    }

    const rawX = clamp(oldX + best.ux * stepDistance, loX, hiX);
    const rawY = clamp(oldY + best.uy * stepDistance, loY, hiY);
    const cleared = pushPursuerOut({ x: rawX, y: rawY }, bars);
    const moved = Math.hypot(cleared.x - oldX, cleared.y - oldY);
    // A proposed step that collapses to near-zero after push-out is a soft collision.
    const hit = !cleared.clear || (moved < stepDistance * 0.15 && stepDistance > 1e-3);
    if (hit) {
      return {
        x: oldX, y: oldY, vx: 0, vy: 0,
        heading: best.angle, blocked: true, hit: true,
      };
    }
    return {
      x: cleared.x,
      y: cleared.y,
      vx: best.ux * speed,
      vy: best.uy * speed,
      heading: best.angle,
      blocked: !best.pathClear,
      hit: false,
    };
  }

  function advanceTarget(episode, dt, bars, rng) {
    if (!episode || dt <= 0) return episode;
    // episode.age drives arena.js's 30 s watchdog: it must measure WALL CLOCK, not
    // target-motion time, or a speed-0 episode (speed slider at its 0 notch) never resets.
    episode.age += dt;
    if (episode.speed <= 1e-6) return episode;
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
      episode.speed, dt, bars, episode.avoidSign, episode.heading
    );
    x = steered.x;
    y = steered.y;
    episode.heading = steered.heading; // persist for ALL patterns, not just cv
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
        // Re-sync the persisted heading, or the next step's continuity window is measured
        // against the pre-bounce heading and the guard fights the bounce.
        episode.heading = Math.atan2(episode.cvVelocity.y, episode.cvVelocity.x);
      }
    }
    return episode;
  }

  return {
    CONTRACT: CONTRACT,
    seededRng: seededRng,
    distanceToBars: distanceToBars,
    pursuerClearance: pursuerClearance,
    segmentPursuerClearance: segmentPursuerClearance,
    sampleWaypoint: sampleWaypoint,
    createEpisode: createEpisode,
    pushOutOfBars: pushOutOfBars,
    pushPursuerOut: pushPursuerOut,
    steerTargetStep: steerTargetStep,
    steerPursuerStep: steerPursuerStep,
    advanceTarget: advanceTarget,
  };
});
