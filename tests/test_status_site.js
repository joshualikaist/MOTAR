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
const Motion = require('../docs/status/arena_motion.js');
const arenaSource = fs.readFileSync(path.join(site, 'arena.js'), 'utf8');

// The public site is one presentation page. JavaScript is limited to the self-contained 3-D
// arena; evidence and claims remain static HTML and never depend on a dashboard renderer.
for (const id of ['arena', 'method', 'evidence', 'platform', 'next']) {
  assert(html.includes(`id="${id}"`), `missing section #${id}`);
}
assert(!css.includes('@import'));
assert(!css.includes('fonts.googleapis.com'));
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
]) assert(html.includes(`${script}?v=20260825`), `stale cache-bust for ${script}`);
assert(html.includes('이 화면은 PPO 실행 영상'));
assert(html.includes('10 Hz 고정 simulation clock'));
assert(html.includes('PhysX 재생'));
assert(html.includes('value="routed-preview" selected'));
assert(html.includes('Physical-style illustration · NOT PhysX'));
assert(html.includes('Global route + bounded/lagged browser preview · NOT PhysX/PPO'));
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
assert(html.includes('Simulation verified'));
assert(html.includes('Hardware pending'));
assert(html.includes('SYNTHETIC_ONLY'));
assert(html.includes('333/333'));
assert(html.includes('−0.0145 pp'));
assert(html.includes('8.443→3.172%'));
assert(html.includes('실제 기체가 미조립'));
assert(readme.includes('Simulation verified, hardware pending'));
assert(readme.includes('실제 센서 로그와 비행 데이터는 없습니다'));
assert(readme.includes('altitude PI → Lee velocity loop'));
assert(fs.readFileSync(path.join(repo, 'docs/assets/motar-control-stack.svg'), 'utf8').includes('K_v e_v'));
assert(html.includes('0.04 s 1차 지연'));

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
