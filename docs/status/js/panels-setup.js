/* setup.html — 초기 세팅. Frozen contract, architecture diagram, 3D arena.
 *
 * applyArenaGeometry and syncArenaToRun live HERE, not with renderLive, because the Live panel is
 * on index.html and the arena is on this page. Both are null-guarded, so leaving them in
 * renderLive would fail silently: the bar slider would sit at its HTML default and the HUD would
 * report a plausible but wrong density with a clean console.
 */

function renderContract(s) {
  const c = s.research_contract || {};
  const task = document.getElementById('contract-task');
  const reward = document.getElementById('contract-reward');
  const warning = document.getElementById('contract-warning');
  const meta = document.getElementById('contract-sub');
  const rows = values => `<tbody>${(values || []).map(r =>
    `<tr><td><b>${r[0]}</b></td><td>${r[1]}</td></tr>`).join('')}</tbody>`;
  if (meta) meta.textContent = c.frozen_policy
    ? `${c.frozen_policy} · checkpoint ${String(c.checkpoint_sha256 || '').slice(0, 12)}…`
    : 'contract unavailable';
  if (task) task.innerHTML = `<thead><tr><th>item</th><th>actual contract</th></tr></thead>${rows(c.task)}`;
  if (reward) reward.innerHTML = `<thead><tr><th>term</th><th>exact form</th></tr></thead>${rows(c.reward)}`;
  if (warning) {
    const a = c.audit || {};
    warning.innerHTML = `<p><b>Frozen checkpoint limitation</b> — ${a.frozen_training || '—'}</p>
      <p><b>Fixed source</b> — ${a.current_source || '—'}</p>
      <p><b>Comparison rule</b> — ${a.comparison_rule || '—'}</p>`;
  }
}

function renderArchitecture(s) {
  const g = s.arena_geometry || {};
  const active = s.active_run && s.active_run.is_live ? s.active_run : (s.latest_run || {});
  const set = (id, value) => {
    const el = document.getElementById(id);
    if (el && value) el.textContent = value;
  };
  const loBars = finite(g.bars_min), hiBars = finite(g.bars_max);
  const speeds = Array.isArray(g.target_speed_m) ? g.target_speed_m.map(finite) : [];
  const arena = finite(g.arena_xy_m), height = finite(g.arena_z_m);
  const lr = finite(active.current_action_learning_rate) || finite(active.recovery_lr) || 5e-6;
  set('arch-bars-range', `막대 장애물 ${loBars == null ? '—' : Math.round(loBars)}–${hiBars == null ? '—' : Math.round(hiBars)}`);
  set('arch-target-speed', speeds.length >= 2 ? `이동 표적 ${speeds[0].toFixed(1)}–${speeds[1].toFixed(1)} m/s` : '이동 표적 —');
  set('arch-arena-contract', arena != null && height != null ? `${arena}×${arena}×${height} m · physics 0.01 s` : 'arena contract —');
  set('arch-lr', lr == null ? 'rl_games · LR —' : `rl_games · LR ${lr.toExponential(0)} 고정`);
}

function applyArenaGeometry(g) {
  if (!g) return;
  if (window.Arena && window.Arena.configure) window.Arena.configure(g);

  const cap = document.getElementById('arena-cap');
  if (cap && g.label) {
    const ep = g.episode_len_steps ? ` · episode ${g.episode_len_steps} steps` : '';
    const gd = g.goal_dist_m ? ` · goal ${g.goal_dist_m[0]}–${g.goal_dist_m[1]} m` : '';
    cap.textContent = `${g.label} · LiDAR 72×4 @12 m · camera 87° @20 m · obstacle tokens 240°${gd}${ep}`;
  }

  const slider = document.getElementById('sl-bars');
  if (slider && g.bars_max) {
    slider.min = String(Math.max(1, Math.round(g.bars_slider_min || 10)));
    slider.max = String(Math.round(g.bars_max));
  }
}

// Point the 3D arena at the campaign the snapshot describes: slider, label, HUD and the
// "current run" preset button.
//
// This used to live at the tail of renderLive. That was harmless while everything shared one page,
// but the Live panel is now on index.html and the arena is on setup.html. Every write below is
// null-guarded, so leaving it in renderLive would fail SILENTLY on setup.html -- the slider would
// sit at its HTML default and the HUD would report a plausible but wrong density, with a clean
// console. Registered from the setup page only.
function syncArenaToRun(s) {
  if (!document.getElementById('stage')) return;   // not the arena page
  const A = s.active_run;
  const L = s.latest_run || {};
  const live = A && A.is_live;
  const E = ((s.research_update || {}).active_experiment || {});
  const finalAudit = !live && E.core_audit_complete === true;
  const riskcapFinal = !live && E.speed_governor_mode === 'riskcap'
    && E.post_evaluation_complete === true && E.generalization_pass === true;

  const nBars = Math.round(live ? A.n_bars_active
    : ((finalAudit || riskcapFinal) ? E.bars : (L.last_n_bars_active || 70)));
  window.__arenaRunBars = nBars;

  const set = (id, fn) => { const el = document.getElementById(id); if (el) fn(el); };
  set('btn-preset', el => { el.textContent = `current run · ${nBars}`; });
  set('sl-bars', el => { el.value = nBars; });
  set('lbl-bars', el => { el.textContent = nBars; });
  set('hud-bars', el => { el.textContent = nBars; });
  set('hud-density', el => { el.textContent = perc100(nBars); });
}

function wireArena() {
  const Arena = window.Arena;
  if (!Arena) return;
  const bars = document.getElementById('sl-bars');
  const speed = document.getElementById('sl-speed');
  const lblB = document.getElementById('lbl-bars');
  const lblS = document.getElementById('lbl-speed');
  const hudB = document.getElementById('hud-bars');
  const hudD = document.getElementById('hud-density');

  const upd = () => {
    const n = +bars.value;
    lblB.textContent = n;
    if (hudB) hudB.textContent = n;
    if (hudD) hudD.textContent = perc100(n);
    Arena.setBars(n);
  };
  if (Number.isFinite(window.__arenaRunBars)) bars.value = window.__arenaRunBars;
  bars.addEventListener('input', upd);
  speed.addEventListener('input', () => {
    const v = +speed.value / 10;
    lblS.textContent = v.toFixed(1);
    Arena.setSpeed(v);
  });

  const play = document.getElementById('btn-play');
  let playing = true;
  play.addEventListener('click', () => {
    playing = !playing;
    Arena.setPlaying(playing);
    play.textContent = playing ? 'pause' : 'play';
    play.setAttribute('aria-pressed', playing ? 'true' : 'false');
  });
  document.getElementById('cb-lidar').addEventListener('change', e => Arena.setLidar(e.target.checked));
  document.getElementById('cb-camera').addEventListener('change', e => Arena.setCamera(e.target.checked));
  document.getElementById('cb-trails').addEventListener('change', e => Arena.setTrails(e.target.checked));
  document.getElementById('btn-view').addEventListener('click', () => Arena.cycleView());
  document.getElementById('btn-preset').addEventListener('click', () => {
    bars.value = Number.isFinite(window.__arenaRunBars) ? window.__arenaRunBars : 70;
    upd();
  });
  upd();
  Arena.setSpeed(+speed.value / 10);
}

MOTAR.register(renderContract, renderArchitecture, syncArenaToRun);
MOTAR.hook(function (s) { if (s && s.arena_geometry) applyArenaGeometry(s.arena_geometry); });

/* 3D boot, after the first render pass has set ARENA_GEO. */
MOTAR.afterBoot(function () {
  requestAnimationFrame(function () {
    var stage = document.getElementById('stage');
    if (!stage) return;
    try {
      if (typeof THREE === 'undefined') throw new Error('THREE missing (vendor/three.min.js)');
      if (!THREE.OrbitControls) throw new Error('OrbitControls missing');
      if (!window.Arena) throw new Error('Arena missing');
      window.__mo = window.Arena;
      // configure BEFORE init: the geometry decides the scene bounds and the initial bar layout
      if (ARENA_GEO) window.Arena.configure(ARENA_GEO);
      window.Arena.init(stage);
      wireArena();
    } catch (e) {
      console.error('3D init', e);
      stage.innerHTML = '<div style="position:absolute;inset:0;display:grid;place-items:center;'
        + 'padding:20px;text-align:center;font:12.5px/1.5 var(--mono),monospace;color:var(--muted)">'
        + '3D init failed: ' + String(e.message || e) + '</div>';
    }
  });
});
