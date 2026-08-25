'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const repo = path.resolve(__dirname, '..');
const site = path.join(repo, 'docs/status');
const html = fs.readFileSync(path.join(site, 'index.html'), 'utf8');
const css = fs.readFileSync(path.join(site, 'style.css'), 'utf8');
const readme = fs.readFileSync(path.join(repo, 'README.md'), 'utf8');
const platform = JSON.parse(fs.readFileSync(path.join(site, 'data/platform.json'), 'utf8'));
const status = JSON.parse(fs.readFileSync(path.join(site, 'status.json'), 'utf8'));
const experiments = JSON.parse(fs.readFileSync(path.join(site, 'data/experiments.json'), 'utf8')).experiments;
const Motion = require('../docs/status/arena_motion.js');
const arenaSource = fs.readFileSync(path.join(site, 'arena.js'), 'utf8');

// The public site is one presentation page. JavaScript is limited to the self-contained 3-D
// arena; evidence and claims remain static HTML and never depend on a dashboard renderer.
for (const id of ['arena', 'method', 'evidence', 'platform', 'next']) {
  assert(html.includes(`id="${id}"`), `missing section #${id}`);
}
assert(!css.includes('@import'));
assert(!css.includes('fonts.googleapis.com'));
assert(html.includes('style.css?v=20260825r3'), 'compact site CSS cache-bust must advance with the layout');
assert(css.includes('height: clamp(360px, 55vh, 620px)'), 'desktop viewer must stay within a viewport-friendly clamp');
assert(css.includes('height: clamp(320px, 72vw, 380px)'), 'mobile viewer must retain a compact height clamp');
assert(css.includes('font-size: 11px'), 'viewer HUD must remain compact');
assert(css.includes('body { margin: 0; overflow-x: hidden; color: var(--ink); background: var(--paper); font: 16px'), 'body text must remain accessible');
for (const retired of [
  'system.html', 'setup.html', 'results.html', 'experiments.html', 'parameters.html', 'drone.html',
  'status.fallback.js', 'js',
]) {
  assert(!fs.existsSync(path.join(site, retired)), `retired dashboard asset remains: ${retired}`);
}
for (const viewerAsset of [
  'arena.js', 'arena_route.js', 'arena_motion.js', 'viewer.js',
  'vendor/three.min.js', 'vendor/OrbitControls.js',
]) assert(fs.existsSync(path.join(site, viewerAsset)), `missing viewer asset: ${viewerAsset}`);
assert(html.indexOf('three.min.js') < html.indexOf('OrbitControls.js'));
assert(html.indexOf('OrbitControls.js') < html.indexOf('arena_route.js'));
assert(html.indexOf('arena_route.js') < html.indexOf('arena_motion.js'));
assert(html.indexOf('arena_motion.js') < html.indexOf('arena.js'));
assert(html.indexOf('arena.js') < html.indexOf('viewer.js'));
for (const script of [
  'vendor/three.min.js', 'vendor/OrbitControls.js', 'arena_route.js',
  'arena_motion.js', 'arena.js', 'viewer.js',
]) assert(html.includes(`${script}?v=20260825r2`), `stale cache-bust for ${script}`);
assert(html.includes('이 화면은 PPO 실행 영상'));
assert(html.includes('10 Hz 고정 simulation clock'));
assert(html.includes('PhysX 재생'));
assert(html.includes('value="routed-preview" selected'));
assert(html.includes('Physical-style illustration · NOT PhysX'));
assert(html.includes('Global route + bounded/lagged browser preview · NOT PhysX/PPO'));
assert(html.includes('half-diagonal support(0.2069 m)'));
assert(html.includes('직전 goal 1.0 m exclusion'));
assert(html.includes('id="hud-route-state"'));
assert(html.includes('id="hud-motion-lineage"'));
assert(arenaSource.includes('routeLine.geometry.setFromPoints'));
assert(arenaSource.includes('route.waypoints.slice(route.cursor)'));
assert(arenaSource.includes('waypointReachM: Motion.CONTRACT.waypointReach'));
assert(arenaSource.includes("document.addEventListener('visibilitychange'"));
assert(arenaSource.includes('drone.rotation.x += (bank - drone.rotation.x)'));
assert(arenaSource.includes('target.rotation.x = previous.targetRoll'));
assert(arenaSource.includes('target.rotation.z = previous.targetPitch'));

Motion.configure({arena_xy_m: 40, goal_dist_m: [6, 28], target_speed_m: [0.3, 1.5]});
assert.deepStrictEqual(Motion.CONTRACT.bounds, {x0: 0, x1: 40, y0: -20, y1: 20});
assert.strictEqual(Motion.CONTRACT.targetDistanceMin, 6);
assert.strictEqual(Motion.CONTRACT.targetDistanceMax, 28);
const episode = Motion.createEpisode(Motion.seededRng(20260824), [], 1.5);
assert(episode.speed >= 0.3 && episode.speed <= 1.5);
episode.speed = 0;
const age = episode.age;
Motion.advanceTarget(episode, 0.1, [], Motion.seededRng(1));
assert.strictEqual(episode.age, age + 0.1, 'speed-zero viewer episodes must still age/reset');

// GitHub README and the site must render the same canonical architecture assets directly.
for (const asset of ['motar-system-overview.svg', 'motar-control-stack.svg']) {
  const assetPath = path.join(repo, 'docs/assets', asset);
  const svg = fs.readFileSync(assetPath, 'utf8');
  assert(svg.includes('<title'));
  assert(svg.includes('<desc'));
  assert(readme.includes(`docs/assets/${asset}`));
  assert(html.includes(`../assets/${asset}`));
}

// Presentation claims remain explicitly bounded by the current evidence and hardware status.
assert(html.includes('Attempt 2 · 32/32 integrity PASS'));
assert(html.includes('Route mechanism FAIL'));
assert(html.includes('Physical PPO BLOCKED'));
assert(html.includes('Hardware pending'));
assert(html.includes('SYNTHETIC_ONLY'));
assert(html.includes('333/333'));
assert(html.includes('selected 205-bar contact endpoints'));
assert(html.includes('−0.0145 pp'));
assert(html.includes('8.443→3.172%'));
assert(html.includes('14.55%'));
assert(html.includes('70-bar · 4-speed pooled plan success'));
assert(html.includes('fallback 35.93% (gate 1%)'));
assert(html.includes('0.25'));
assert(html.includes('70 bars × 0.6 m/s'));
assert(html.includes('unsafe_start'));
assert(html.includes('NOT PhysX/PPO'));
assert(html.includes('실제 기체가 미조립'));
assert(readme.includes('Status · 2026-08-25'));
assert(readme.includes('32/32 integrity checks'));
assert(readme.includes('route mechanism **failed**'));
assert(readme.includes('physical PPO and hardware claims\n> remain **blocked**'));
assert(readme.includes('0.25 goals/env'));
assert(readme.includes('`unsafe_start` recovery'));
assert(readme.includes('Motor saturation, tilt, and contact'));
assert(readme.includes('gates passed, so they are not the supported explanation'));
assert(readme.includes('altitude PI → Lee velocity loop'));
assert(fs.readFileSync(path.join(repo, 'docs/assets/motar-control-stack.svg'), 'utf8').includes('K_v e_v'));
assert(html.includes('0.04 s 1차 지연'));

const routedGate = experiments.find((entry) => entry.id === '2026-08-25-physical-target-routed-simulator-gate-seed827-attempt2');
assert(routedGate, 'canonical attempt-2 routed gate entry missing');
assert.strictEqual(routedGate.verdict, 'FAIL');
assert.strictEqual(routedGate.validity, 'canonical');
assert.strictEqual(routedGate.arms[0].extra.cells_passed, '32/32');
assert(routedGate.verdict_note.includes('unsafe_start'));
assert.strictEqual(status.sim2real_72h.simulation_verification.preflight_steps.physical_target_gate.integrity, 'PASS_32_CELL_INTEGRITY');
assert.strictEqual(status.sim2real_72h.simulation_verification.preflight_steps.physical_target_gate.route_mechanism, 'FAIL_ROUTE_MECHANISM');
assert.strictEqual(status.sim2real_72h.simulation_verification.preflight_steps.physical_target_gate.physical_ppo, 'BLOCKED');
assert.deepStrictEqual(
  status.sim2real_72h.simulation_verification.routed_physical_target_gate_attempt2.highest_passing_speed_mps_by_density,
  {'70': null, '150': null, '205': null, '300': null},
);
assert.strictEqual(
  status.sim2real_72h.simulation_verification.historical_post_wall_brake_speed_envelope.route_mode,
  'off_historical_lineage',
);
assert(status.sim2real_72h.status.includes('ROUTE MECHANISM FAILED'));
assert(status.sim2real_72h.status.includes('PHYSICAL PPO BLOCKED'));

// The concise platform card must remain tied to the generated source-of-truth values.
const ref = platform.robots.find((robot) => robot.key === 'navrl_ref5in_quad');
assert(ref);
assert.strictEqual(ref.mass_kg, 1.2);
assert.strictEqual(ref.derived.motor_diagonal_m, 0.22);
assert.deepStrictEqual(ref.collision_box_m, [0.28, 0.28, 0.12]);
assert(html.includes('1.20 kg'));
assert(html.includes('220 mm'));
assert(html.includes('0.28 × 0.28 × 0.12 m'));

// Every local href and image source resolves from the static page.
const refs = [];
const refPattern = /(?:href|src)="([^"]+)"/g;
let match;
while ((match = refPattern.exec(html)) !== null) refs.push(match[1]);
for (const refPath of refs) {
  if (refPath.startsWith('#') || /^https?:/.test(refPath)) continue;
  const clean = refPath.split('#')[0].split('?')[0];
  assert(fs.existsSync(path.resolve(site, clean)), `broken local reference: ${refPath}`);
}

console.log('MOTAR static site contract: PASS');
