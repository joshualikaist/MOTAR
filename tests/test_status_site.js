'use strict';

const assert = require('assert');
const { execFileSync } = require('child_process');
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
const trackedFiles = new Set(
  execFileSync('git', ['ls-files', '--cached', '-z'], {
    cwd: repo,
    encoding: 'utf8',
  }).split('\0').filter(Boolean),
);
const pendingDocs = new Set(
  execFileSync('git', ['ls-files', '--others', '--exclude-standard', '-z'], {
    cwd: repo,
    encoding: 'utf8',
  }).split('\0').filter((file) => file.startsWith('docs/')),
);

// The public site is one presentation page. JavaScript is limited to the self-contained 3-D
// arena; evidence and claims remain static HTML and never depend on a dashboard renderer.
for (const id of ['arena', 'method', 'evidence', 'platform', 'next']) {
  assert(html.includes(`id="${id}"`), `missing section #${id}`);
}
assert(!css.includes('@import'));
assert(!css.includes('fonts.googleapis.com'));
assert(html.includes('style.css?v=20260827r6'), 'compact site CSS cache-bust must advance with the layout');
assert(css.includes('height: clamp(300px, 44vh, 500px)'), 'desktop viewer must stay within a viewport-friendly clamp');
assert(css.includes('height: clamp(260px, 64vw, 340px)'), 'mobile viewer must retain a compact height clamp');
assert(css.includes('font-size: 11px'), 'viewer HUD must remain compact');
assert(css.includes('body { margin: 0; overflow-x: hidden; color: var(--ink); background: var(--paper); font: 15px'), 'body text must remain compact and readable');
assert(css.includes('word-break: break-word'), 'mobile authority status must not clip long slash-delimited tokens');
assert(html.includes('../research_authority_2026-08-26.json'), 'frozen authority receipt must be linked');
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
]) assert(html.includes(`${script}?v=20260827r6`), `stale cache-bust for ${script}`);
assert(html.includes('이 화면은 PPO 실행 영상'));
assert(html.includes('10 Hz 고정 simulation clock'));
assert(html.includes('PhysX 재생'));
assert(html.includes('value="routed-preview" selected'));
assert(html.includes('historical <code>global_astar_v1</code>'));
assert(html.includes('recovery-v2 state machine'));
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
assert(html.includes('Non-overlap physical lineage · 205-bar geometry PASS'));
assert(html.includes('Historical navrl_band results retained but not reusable'));
assert(html.includes('overlap 0'));
assert(html.includes('fresh PPO required'));
assert(html.includes('99.167%'));
assert(html.includes('300 bars disconnected FAIL'));
assert(html.includes('70→205 train · 300 disconnected'));
assert(html.includes('preregistration_navrl_v2_corrected_density_geometry_2026-08-27.md'));
assert(html.includes('SYNTHETIC_ONLY'));
assert(html.includes('333/333'));
assert(html.includes('selected 205-bar contact endpoints'));
assert(html.includes('−0.0145 pp'));
assert(html.includes('8.443→3.172%'));
assert(html.includes('190/203'));
assert(html.includes('93.60%'));
assert(html.includes('18,381/38,400'));
assert(html.includes('47.87%'));
assert(html.includes('0.21875'));
assert(html.includes('70 bars × 0.6 m/s'));
assert(html.includes('96,854/153,600'));
assert(html.includes('63.06%'));
assert(html.includes('0/534'));
assert(html.includes('primary_n=1'));
assert(html.includes('identity_void=false'));
assert(html.includes('certificate 49 + brake_timeout 32 +'));
assert(html.includes('resume 23 + connect_timeout 1 + brake_no_anchor 1'));
assert(html.includes('baseline_1p25'));
assert(html.includes('canonical 1.5 m/s'));
assert(html.includes('RANGE_INCONCLUSIVE'));
assert(html.includes('추가 Track B'));
assert(html.includes('RECOVERY_DOMINANT'));
assert(html.includes('NOT PhysX/PPO'));
assert(html.includes('실제 기체가 미조립'));
assert(fs.readFileSync(path.join(repo, 'docs/assets/motar-control-stack.svg'), 'utf8').includes('K_v e_v'));
assert(html.includes('0.04 s 1차 지연'));

const recoveryV2Gate = experiments.find((entry) => entry.id === '2026-08-26-physical-target-recovery-v2-lower1p25-gate-seed827');
assert(recoveryV2Gate, 'canonical recovery-v2 lower-1.25 gate entry missing');
assert.strictEqual(recoveryV2Gate.verdict, 'FAIL');
assert.strictEqual(recoveryV2Gate.validity, 'canonical');
assert.strictEqual(recoveryV2Gate.env.contract_variant, 'baseline_1p25');
assert.strictEqual(recoveryV2Gate.arms[0].extra.cells_passed, '7/32, all route-off');
assert(recoveryV2Gate.arms[1].extra.plan_success.includes('190/203'));
assert(recoveryV2Gate.arms[1].extra.fallback.includes('18381/38400'));
assert(recoveryV2Gate.verdict_note.includes('canonical 1.5'));
assert(recoveryV2Gate.verdict_note.includes('hardware'));

const noAnchor = experiments.find((entry) => entry.id === '2026-08-26-recovery-v2-no-connector-forensics-seed827');
assert(noAnchor, 'canonical recovery-v2 no-anchor entry missing');
assert.strictEqual(noAnchor.verdict, 'INCONCLUSIVE');
assert.strictEqual(noAnchor.validity, 'canonical');
assert.strictEqual(noAnchor.arms[0].extra.primary_n, 1);
assert.strictEqual(noAnchor.arms[0].extra.identity_void, false);
assert.strictEqual(noAnchor.arms[1].extra.failed_certificate, 49);
assert.strictEqual(noAnchor.arms[1].extra.brake_timeout, 32);
assert.strictEqual(noAnchor.arms[1].extra.failed_resume, 23);
assert.strictEqual(noAnchor.arms[1].extra.connect_timeout, 1);
assert.strictEqual(noAnchor.arms[1].extra.brake_no_anchor, 1);

// Attempt 2 and RECOVERY_DOMINANT remain inspectable historical lineage.
const routedGate = experiments.find((entry) => entry.id === '2026-08-25-physical-target-routed-simulator-gate-seed827-attempt2');
assert(routedGate, 'canonical attempt-2 routed gate entry missing');
assert.strictEqual(routedGate.verdict, 'FAIL');
assert.strictEqual(routedGate.validity, 'canonical');
assert.strictEqual(routedGate.lineage_status, 'historical_attempt2');
assert.strictEqual(routedGate.arms[0].extra.cells_passed, '32/32');
assert(routedGate.verdict_note.includes('unsafe_start'));
const recovery = status.sim2real_72h.simulation_verification.preflight_steps.route_recovery_forensics;
assert(recovery, 'route recovery forensics status missing');
assert.strictEqual(recovery.diagnostic_verdict, 'RECOVERY_DOMINANT');
assert.strictEqual(recovery.lineage_status, 'HISTORICAL_V1_DIAGNOSTIC');
assert.strictEqual(recovery.cells_verified, '8/8');
assert.strictEqual(recovery.local_invalidations, 358);
assert.strictEqual(recovery.local_fallback_intervals, 35666);
assert.strictEqual(recovery.unique_local_origins, 200);
assert.strictEqual(recovery.rounded_vs_square_disagreements, 1832);
assert.strictEqual(recovery.margin_tuning_allowed, false);
const recoveryExperiment = experiments.find((entry) => entry.id === '2026-08-25-physical-target-route-recovery-forensics-seed827');
assert(recoveryExperiment, 'recovery forensics experiment entry missing');
assert.strictEqual(recoveryExperiment.diagnostic_verdict, 'RECOVERY_DOMINANT');
assert.strictEqual(recoveryExperiment.lineage_status, 'historical_v1_diagnostic');
assert(recoveryExperiment.results_paths.includes('results/navrl_physical_target_route_recovery_forensics_seed827/receipt.json'));

assert.strictEqual(status.sim2real_72h.as_of, '2026-08-26');
const currentGate = status.sim2real_72h.simulation_verification.recovery_v2_lower1p25_gate;
assert.strictEqual(currentGate.integrity, 'PASS_32_CELL_INTEGRITY');
assert.strictEqual(currentGate.route_mechanism, 'FAIL_ROUTE_MECHANISM');
assert.deepStrictEqual(currentGate.cells, {
  passed: 7,
  total: 32,
  route_off_passed: 7,
  route_off_total: 16,
  recovery_passed: 0,
  recovery_total: 16,
  passing_lineage: 'route_off_only',
});
assert.strictEqual(currentGate.plan_success_70bar_4speed.numerator, 190);
assert.strictEqual(currentGate.plan_success_70bar_4speed.denominator, 203);
assert.strictEqual(currentGate.fallback_70bar_4speed.numerator, 18381);
assert.strictEqual(currentGate.fallback_70bar_4speed.denominator, 38400);
assert.strictEqual(currentGate.goals_per_env_70bar_0_6mps.value, 0.21875);
assert.strictEqual(currentGate.no_connector_occupancy.numerator, 96854);
assert.strictEqual(currentGate.no_connector_occupancy.denominator, 153600);
assert.strictEqual(currentGate.hard_breach_no_connector_entries.numerator, 0);
assert.strictEqual(currentGate.hard_breach_no_connector_entries.denominator, 534);
assert.strictEqual(currentGate.hardware_claim, false);
assert.strictEqual(currentGate.canonical_1p5_contract, 'SEPARATE_UNCHANGED_NOT_PASSED');
assert.deepStrictEqual(
  status.sim2real_72h.simulation_verification.preflight_steps.physical_target_gate,
  currentGate,
);
const currentForensics = status.sim2real_72h.simulation_verification.recovery_v2_no_connector_forensics;
assert.strictEqual(currentForensics.decision_rule.label, 'INCONCLUSIVE');
assert.strictEqual(currentForensics.decision_rule.primary_n, 1);
assert.strictEqual(currentForensics.decision_rule.anchor_present, 0);
assert.strictEqual(currentForensics.decision_rule.hard_free_soft_unsafe, 1);
assert.strictEqual(currentForensics.decision_rule.identity_void, false);
assert.strictEqual(currentForensics.no_connector_classes.total, 106);
assert.strictEqual(
  status.sim2real_72h.simulation_verification.track_b_authority,
  'CLOSED_NO_FURTHER_GPU_PPO_RETUNE_RERUN',
);
assert.deepStrictEqual(
  status.sim2real_72h.simulation_verification.routed_physical_target_gate_attempt2.highest_passing_speed_mps_by_density,
  {'70': null, '150': null, '205': null, '300': null},
);
assert.strictEqual(
  status.sim2real_72h.simulation_verification.historical_post_wall_brake_speed_envelope.route_mode,
  'off_historical_lineage',
);
assert(status.sim2real_72h.status.includes('NO FURTHER TRACK B AUTHORITY'));
assert(status.sim2real_72h.status.includes('HARDWARE NEXT'));

// The concise platform card must remain tied to the generated source-of-truth values.
const ref = platform.robots.find((robot) => robot.key === 'navrl_ref5in_quad');
assert(ref);
assert.strictEqual(ref.mass_kg, 1.2);
assert.strictEqual(ref.derived.motor_diagonal_m, 0.22);
assert.deepStrictEqual(ref.collision_box_m, [0.28, 0.28, 0.12]);
const refV2 = platform.robots.find((robot) => robot.key === 'navrl_ref5in_v2_quad');
assert(refV2);
assert.deepStrictEqual(refV2.collision_box_m, [0.283, 0.283, 0.12]);
assert(html.includes('1.20 kg'));
assert(html.includes('220 mm'));
assert(html.includes('0.283 × 0.283 × 0.12 m'));

// Every local href and image source resolves from the static page.
const refs = [];
const refPattern = /(?:href|src)="([^"]+)"/g;
let match;
while ((match = refPattern.exec(html)) !== null) refs.push(match[1]);
for (const refPath of refs) {
  if (refPath.startsWith('#') || /^https?:/.test(refPath)) continue;
  const clean = refPath.split('#')[0].split('?')[0];
  const target = path.resolve(site, clean);
  assert(fs.existsSync(target), `broken local reference: ${refPath}`);
  const relative = path.relative(repo, target);
  assert(
    !relative.startsWith('..') &&
      !path.isAbsolute(relative) &&
      (trackedFiles.has(relative) || pendingDocs.has(relative)),
    `local reference is not tracked or a pending docs file: ${refPath}`,
  );
}

console.log('MOTAR static site contract: PASS');
