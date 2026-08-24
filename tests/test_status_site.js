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

// The public site is deliberately one static page: no dashboard runtime, duplicated subpages,
// remote fonts, or JavaScript-rendered evidence.
for (const id of ['method', 'evidence', 'platform', 'next']) {
  assert(html.includes(`id="${id}"`), `missing section #${id}`);
}
assert(!html.includes('<script'));
assert(!css.includes('@import'));
assert(!css.includes('fonts.googleapis.com'));
for (const retired of [
  'system.html', 'setup.html', 'results.html', 'experiments.html', 'parameters.html', 'drone.html',
  'arena.js', 'arena_motion.js', 'status.fallback.js', 'js', 'vendor',
]) {
  assert(!fs.existsSync(path.join(site, retired)), `retired dashboard asset remains: ${retired}`);
}

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
