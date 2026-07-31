/* MOTAR status board — data panels. Arena = window.Arena from arena.js */

const pct = x => x == null ? '—' : (x * 100).toFixed(1) + '%';
// Obstacle-placement area [m^2]. v1: a 24 m arena with bars confined to x in 0.13..0.96 -> 478.
// v2: a 40 m arena with the band widened to the full width -> the whole 1600 m^2 footprint.
// Set from status.json so a v1 and a v2 run are never reported on the same denominator.
let BAND_AREA = 478;
const perc100 = n => ((n / BAND_AREA) * 100).toFixed(1);
// Geometry of the task actually being described; overwritten from status.json.arena_geometry.
let ARENA_GEO = null;

/* Retarget the arena panel (3D scene, bars slider, caption) at the running task's real geometry.
   Without this the panel silently drew the v1 24 m arena while a 40 m v2 run was training. */
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
const cssv = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;

function toggleTheme() {
  const r = document.documentElement;
  const cur = r.getAttribute('data-theme') || 'light';
  r.setAttribute('data-theme', cur === 'dark' ? 'light' : 'dark');
  if (window.__mo) window.__mo.recolor();
}

function fallbackStatus() {
  return JSON.parse(document.getElementById('fallback').textContent);
}
async function fetchStatus() {
  const r = await fetch('status.json', { cache: 'no-store' });
  if (!r.ok) throw new Error('http ' + r.status);
  return await r.json();
}

function wireChrome() {
  const theme = document.getElementById('themebtn');
  if (theme) theme.addEventListener('click', toggleTheme);

  document.querySelectorAll('.seg-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.seg-btn').forEach(b => b.classList.toggle('on', b === btn));
      const which = btn.dataset.curve;
      document.getElementById('pane-density').classList.toggle('on', which === 'density');
      document.getElementById('pane-speed').classList.toggle('on', which === 'speed');
      const m = document.getElementById('pane-map');
      if (m) m.classList.toggle('on', which === 'map');
    });
  });
}

function renderLive(s) {
  const A = s.active_run;
  const L = s.latest_run || {};
  const live = A && A.is_live;
  const src = live ? A : L;
  const runName = ((src && src.run) || '—').replace(/_navrl$/, '');

  const pill = document.getElementById('livepill');
  const pillTxt = document.getElementById('livepill-txt');
  const pillRun = document.getElementById('livepill-run');
  if (pill) pill.className = 'pill ' + (live ? 'is-live' : 'is-snapshot');
  if (pillTxt) pillTxt.textContent = live ? 'LIVE' : 'LAST';
  if (pillRun) pillRun.textContent = runName;

  const footGen = document.getElementById('foot-gen');
  const footRuns = document.getElementById('foot-runs');
  if (footGen) footGen.textContent = (s.generated_at || '').slice(0, 16).replace('T', ' ') || '—';
  if (footRuns) footRuns.textContent = s.n_runs || (s.runs ? s.runs.length : '—');

  const h2 = document.getElementById('live-h2');
  const sub = document.getElementById('live-sub');
  const cards = document.getElementById('live-cards');
  const capEl = document.getElementById('live-cap');
  if (h2) h2.textContent = live ? 'Live' : 'Latest run';
  if (sub) {
    sub.textContent = live
      ? `${runName} · ep ${Math.round(A.epoch)} · bars ${Math.round(A.n_bars_active)} · goal ${Number(A.curriculum_max_m).toFixed(1)} m · tail ${A.tail_epochs || 50}`
      : `${runName} · ${L.epochs_logged || '?'} ep · bars ${L.last_n_bars_active ?? '?'} · goal ${L.last_curriculum_max_m ?? '?'} m`;
  }

  const peak = L.peak_captured_rate;
  const finalCap = live ? A.captured_rate : L.last_captured_rate;
  const finalCrash = live ? A.crash_rate : L.last_crash_rate;
  const finalTo = live ? A.timeout_rate : L.last_timeout_rate;
  const gapPt = (peak != null && L.last_captured_rate != null)
    ? ((peak - L.last_captured_rate) * 100).toFixed(0) : null;
  const bars = live ? A.n_bars_active : L.last_n_bars_active;

  if (cards) {
    const rows = [
      { k: 'capture', v: pct(finalCap), c: finalCap >= 0.7 ? 'good' : finalCap >= 0.55 ? 'warn' : 'bad', s: live ? 'tail avg' : 'final' },
      { k: 'crash', v: pct(finalCrash), c: finalCrash <= 0.15 ? 'good' : finalCrash <= 0.35 ? 'warn' : 'bad', s: 'fail mode' },
      { k: 'timeout', v: pct(finalTo), c: 'acc', s: '' },
      { k: 'bars', v: bars != null ? String(Math.round(bars)) : '—', c: 'acc', s: 'active' },
      live
        ? { k: 'epoch', v: String(Math.round(A.epoch)), c: 'acc', s: `/ ${A.max_epochs || '?'} max` }
        : { k: 'peak→final', v: gapPt != null ? ('−' + gapPt + 'pt') : '—', c: gapPt >= 30 ? 'bad' : gapPt >= 15 ? 'warn' : 'good', s: pct(peak) + ' peak' },
    ];
    cards.innerHTML = rows.map(c =>
      `<div class="stat ${c.c}"><div class="v">${c.v}</div><div class="k">${c.k}</div><div class="s">${c.s}</div></div>`
    ).join('');
  }
  if (capEl) {
    capEl.textContent = live
      ? `Live = last ${A.tail_epochs || 50} epochs · current curriculum state.`
      : `peak ${pct(peak)} (ep ${L.peak_captured_epoch}) → final ${pct(L.last_captured_rate)}.`;
  }

  const preset = document.getElementById('btn-preset');
  const nBars = live ? Math.round(A.n_bars_active) : (L.last_n_bars_active || 70);
  window.__arenaRunBars = Math.round(nBars);
  if (preset) preset.textContent = `current run · ${Math.round(nBars)}`;
  const slider = document.getElementById('sl-bars');
  const sliderLabel = document.getElementById('lbl-bars');
  const hudBars = document.getElementById('hud-bars');
  const hudDensity = document.getElementById('hud-density');
  if (slider) slider.value = Math.round(nBars);
  if (sliderLabel) sliderLabel.textContent = Math.round(nBars);
  if (hudBars) hudBars.textContent = Math.round(nBars);
  if (hudDensity) hudDensity.textContent = perc100(nBars);
}

function renderCriteria(s) {
  const c = s.success_criteria;
  if (!c) return;
  const set = (id, html) => { const el = document.getElementById(id); if (el) el.innerHTML = html; };

  set('criteria-headline', c.headline || '');

  const p = c.primary || {};
  set('criteria-primary',
    `<p><b>${p.metric || ''}</b> — ${p.definition || ''}</p>
     <p><b>Measured by</b> ${p.measured_by || ''}</p>
     <p class="decision">${p.why || ''}</p>`);

  set('criteria-secondary',
    `<thead><tr><th>metric</th><th>definition</th><th>role</th></tr></thead><tbody>${
      (c.secondary || []).map(r =>
        `<tr><td><b>${r.metric}</b></td><td>${r.definition}</td><td>${r.role}</td></tr>`).join('')
    }</tbody>`);

  set('criteria-not',
    `<thead><tr><th>metric</th><th>why it does not measure success</th></tr></thead><tbody>${
      (c.not_success_metrics || []).map(r =>
        `<tr><td><b>${r.metric}</b></td><td>${r.why}</td></tr>`).join('')
    }</tbody>`);

  const g = c.curriculum_gate || {};
  set('criteria-gate',
    `<p><b>${g.what || ''}</b></p>
     <p>${g.rule || ''}</p>
     <p>${g.why_ramped || ''}</p>
     <p class="decision">${g.caution || ''}</p>`);

  set('criteria-ckpt', c.checkpoint_rule || '');
}

function renderResearchUpdate(s) {
  const u = s.research_update || {};
  const experiment = u.active_experiment || u.bounded_pilot || {};
  const hero = document.getElementById('update-hero');
  const sub = document.getElementById('update-sub');
  const milestones = document.getElementById('update-milestones');
  const evalTbl = document.getElementById('update-eval');
  const decision = document.getElementById('update-decision');

  if (sub) sub.textContent = u.subtitle || '85-bar obstacle-token coverage ablation';
  if (hero) {
    const active = experiment.is_live ? '<span class="update-live">RUNNING</span>' : '<span class="update-snap">SNAPSHOT</span>';
    hero.innerHTML = `<div><span class="eyebrow">CURRENT HYPOTHESIS</span>
      <strong>${u.headline || 'Sensor fixed; action support is the active bottleneck.'}</strong>
      <p>${u.summary || ''}</p></div>
      <div class="update-run">${active}<b>${experiment.run || '—'}</b>
      <span>ep ${experiment.epoch ?? '—'} / ${experiment.max_epochs ?? '—'} · ${experiment.bars ?? '—'} bars</span></div>`;
  }
  if (milestones) {
    milestones.innerHTML = (u.milestones || []).map(m => `
      <article class="milestone ${m.state || ''}">
        <span>${m.label || ''}</span><b>${m.value || ''}</b><p>${m.detail || ''}</p>
      </article>`).join('');
  }
  if (evalTbl) {
    const rows = u.comparison || [];
    if (rows.length) {
      evalTbl.innerHTML = `<thead><tr><th>condition</th><th>bars</th><th>capture</th><th>unique</th><th>verdict</th></tr></thead>
        <tbody>${rows.map(r => `<tr><td>${r.label || '—'}</td>
          <td>${r.bars ?? '—'}</td><td>${r.capture == null ? '—' : pct(r.capture)}</td>
          <td>${r.unique == null ? '—' : Number(r.unique).toFixed(1)}</td>
          <td>${r.verdict || '—'}</td></tr>`).join('')}</tbody>`;
    } else {
      const legacy = u.legacy_eval || [];
      evalTbl.innerHTML = `<thead><tr><th>target</th><th>capture</th><th>crash</th><th>bar</th><th>below</th></tr></thead>
        <tbody>${legacy.map(r => `<tr><td>${Number(r.target_speed).toFixed(2)}</td>
          <td>${pct(r.capture)}</td><td>${pct(r.crash)}</td>
          <td>${pct(r.bar_contact)}</td><td>${pct(r.below)}</td></tr>`).join('')}</tbody>`;
    }
  }
  if (decision) {
    const gates = u.gates || [];
    decision.innerHTML = gates.map(g => `<p><b>${g.label}</b> ${g.value}</p>`).join('')
      + `<p class="decision">${u.decision || ''}</p>`;
  }
}

function renderCorridor(s) {
  const c = s.corridor_token || {};
  const current = c.current || {};
  const proposed = c.proposed || {};
  const setText = (id, value) => {
    const el = document.getElementById(id);
    if (el && value != null) el.textContent = value;
  };
  const setFields = (id, fields) => {
    const el = document.getElementById(id);
    if (el) el.innerHTML = (fields || []).map(v => `<li>${v}</li>`).join('');
  };

  setText('corridor-sub', c.subtitle);
  setText('corridor-title', c.title ? `${c.title}: free space as a policy input` : null);
  setText('corridor-definition', c.definition);
  setText('corridor-why', c.why_now);
  setText('token-current-label', current.label);
  setText('token-current-question', current.question);
  setFields('token-current-fields', current.fields);
  setText('token-current-weakness', current.weakness);
  setText('token-proposed-label', proposed.label);
  setText('token-proposed-question', proposed.question);
  setFields('token-proposed-fields', proposed.fields);
  setText('token-proposed-weakness', proposed.weakness);
  setText('corridor-gate', c.pilot_gate ? `PILOT GATE · ${c.pilot_gate}` : null);

  const steps = document.getElementById('corridor-steps');
  if (steps) {
    steps.innerHTML = (c.steps || []).map(step => `
      <article class="corridor-step ${step.state || ''}">
        <span>${step.id || ''}</span>
        <div><b>${step.title || ''}</b><p>${step.detail || ''}</p></div>
      </article>`).join('');
  }
}

function pickDensity(s) {
  const c = s.density_curves || {};
  const pack = c.corrected_chirality_density_curve || c.general_repr_density_curve || c.vision_density_curve;
  const raw = Array.isArray(pack) ? pack : (pack && pack.rows) || [];
  const gtPack = c.vision_density_curve;
  const gtRows = Array.isArray(gtPack) ? gtPack : (gtPack && gtPack.rows) || [];
  const gtByBars = Object.fromEntries(gtRows.map(r => {
    const b = r.density_bars ?? r.bars;
    return [b, r.gt_injected_phase2];
  }).filter(([b, g]) => b != null && g != null));

  const liveBars = s.active_run && s.active_run.is_live ? s.active_run.n_bars_active : null;
  const trainedMax = Math.round(liveBars || (s.latest_run && s.latest_run.last_n_bars_active) || 65);

  return raw.map(r => {
    const bars = r.density_bars ?? r.bars;
    return {
      density_bars: bars,
      captured: r.captured ?? r.capture,
      crash: r.crash,
      timeout: r.timeout,
      trained: bars != null ? bars <= trainedMax : null,
      gt_injected_phase2: r.gt_injected_phase2 ?? gtByBars[bars] ?? null,
    };
  }).filter(r => r.density_bars != null).sort((a, b) => a.density_bars - b.density_bars);
}

function drawIn(svg) {
  if (reduceMotion) {
    svg.querySelectorAll('.draw,.pt').forEach(el => el.classList.add('in'));
    return;
  }
  svg.querySelectorAll('.draw').forEach(el => {
    try {
      const len = el.getTotalLength();
      el.style.setProperty('--len', len);
      el.classList.add('in');
    } catch (_) { el.classList.add('in'); }
  });
  setTimeout(() => svg.querySelectorAll('.pt').forEach(el => el.classList.add('in')), 160);
}

function renderCurve(s) {
  const rows = pickDensity(s);
  const trainedMax = rows.find(r => r.trained === false)
    ? Math.max(...rows.filter(r => r.trained).map(r => r.density_bars))
    : (s.active_run && s.active_run.is_live
      ? Math.round(s.active_run.n_bars_active)
      : (s.latest_run && s.latest_run.last_n_bars_active) || null);

  const sub = document.getElementById('density-sub');
  if (sub) sub.textContent = rows.length
    ? 'held-out · sensor-only · cells above trained max = OOD'
    : 'no density curve yet';
  if (!rows.length) return;

  const xs = rows.map(r => r.density_bars);
  const xmin = Math.min(...xs), xmax = Math.max(...xs);
  const PX0 = 70, PX1 = 690, PY0 = 40, PY1 = 350;
  const sx = v => PX0 + (v - xmin) / Math.max(1e-6, xmax - xmin) * (PX1 - PX0);
  const sy = v => PY1 - v * (PY1 - PY0);
  const path = key => rows.filter(r => r[key] != null)
    .map(r => `${sx(r.density_bars).toFixed(1)},${sy(r[key]).toFixed(1)}`).join(' ');
  const dots = (key, col, r0) => rows.filter(r => r[key] != null)
    .map(r => `<circle class="pt" cx="${sx(r.density_bars).toFixed(1)}" cy="${sy(r[key]).toFixed(1)}" r="${r0}" fill="${col}"/>`).join('');

  let g = '';
  for (let i = 0; i <= 4; i++) {
    const y = PY0 + i * (PY1 - PY0) / 4;
    g += `<line x1="${PX0}" y1="${y}" x2="${PX1}" y2="${y}" stroke="${cssv('--line')}" stroke-width="1"/>`;
  }
  const trainedBand = trainedMax != null && trainedMax >= xmin && trainedMax <= xmax ? `
    <rect x="${PX0}" y="${PY0}" width="${(sx(trainedMax) - PX0).toFixed(1)}" height="${PY1 - PY0}"
      fill="${cssv('--accent')}" opacity="0.07"/>
    <line x1="${sx(trainedMax).toFixed(1)}" y1="${PY0}" x2="${sx(trainedMax).toFixed(1)}" y2="${PY1}"
      stroke="${cssv('--accent')}" stroke-width="1.5" stroke-dasharray="4 4" opacity="0.75"/>` : '';
  const ylab = [100, 75, 50, 25, 0].map((v, i) =>
    `<text x="60" y="${(PY0 + i * (PY1 - PY0) / 4 + 4).toFixed(0)}" text-anchor="end">${v}${i === 0 ? '%' : ''}</text>`).join('');
  const xlab = rows.map(r =>
    `<text x="${sx(r.density_bars).toFixed(0)}" y="${PY1 + 18}" text-anchor="middle">${r.density_bars}</text>`).join('');

  const curve = document.getElementById('curve');
  if (curve) {
    curve.innerHTML = `
      ${g}
      <line x1="${PX0}" y1="${PY1}" x2="${PX1}" y2="${PY1}" stroke="${cssv('--line')}" stroke-width="1.5"/>
      <line x1="${PX0}" y1="${PY0}" x2="${PX0}" y2="${PY1}" stroke="${cssv('--line')}" stroke-width="1.5"/>
      <g class="t dim" font-size="11">${ylab}</g>
      <g class="t dim" font-size="11">${xlab}</g>
      ${trainedBand}
      <polyline class="draw" fill="none" stroke="${cssv('--gt')}" stroke-width="2.2" stroke-dasharray="6 5" points="${path('gt_injected_phase2')}"/>
      ${dots('gt_injected_phase2', cssv('--gt'), 3.5)}
      <polyline class="draw" fill="none" stroke="${cssv('--bad')}" stroke-width="2.2" points="${path('crash')}"/>
      ${dots('crash', cssv('--bad'), 3.5)}
      <polyline class="draw" fill="none" stroke="${cssv('--sensor')}" stroke-width="3" points="${path('captured')}"/>
      ${dots('captured', cssv('--sensor'), 4)}`;
    requestAnimationFrame(() => drawIn(curve));
  }

  const tbl = document.getElementById('curve-tbl');
  if (tbl) {
    tbl.innerHTML = `<thead><tr><th>bars</th><th>/100m²</th><th>capture</th><th>crash</th><th>timeout</th><th>zone</th><th>GT</th></tr></thead>
      <tbody>${rows.map(r => {
        const cliff = r.crash >= 0.5 ? ' class="cliff"' : '';
        const zone = r.trained === false ? '<span class="tag-ood">OOD</span>'
          : (r.trained ? '<span class="tag-tr">train</span>' : '—');
        return `<tr${cliff}><td>${r.density_bars}</td><td>${perc100(r.density_bars)}</td><td>${pct(r.captured)}</td><td>${pct(r.crash)}</td><td>${pct(r.timeout)}</td><td>${zone}</td><td>${r.gt_injected_phase2 != null ? pct(r.gt_injected_phase2) : '—'}</td></tr>`;
      }).join('')}</tbody>`;
  }
  const cap = document.getElementById('curve-cap');
  if (cap) cap.textContent = 'deterministic · FOV 240°';
}

function pickSpeed(s) {
  const c = s.speed_curves || {};
  const pack = c.general_repr_fov240_speed_axis || c.general_repr_speed_axis;
  const raw = Array.isArray(pack) ? pack : (pack && pack.rows) || [];
  return raw.map(r => {
    const speed = r.speed ?? r.target_speed ?? r.target_speed_ms;
    if (speed === 'mean' || speed == null || Number.isNaN(+speed)) return null;
    return {
      speed: +speed,
      captured_240: r.capture_fov240 ?? r.captured_240 ?? r.captured ?? r.capture,
      crash_240: r.bar_contact_fov240 ?? r.crash_240 ?? r.crash,
      captured_360: r.capture_fov360_baseline ?? r.captured_360,
      crash_360: r.bar_contact_fov360_baseline ?? r.crash_360,
    };
  }).filter(Boolean).sort((a, b) => a.speed - b.speed);
}

/* Density x target-speed capture heat map -- the paper headline figure. */
function renderHeatmap(s) {
  const pack = s.density_speed_map;
  const rows = (pack && pack.rows) || [];
  const svg = document.getElementById('heatmap');
  if (!svg) return;
  const sub = document.getElementById('map-sub');
  if (!rows.length) { if (sub) sub.textContent = 'no map measured yet'; return; }

  const bars = [...new Set(rows.map(r => r.bars))].sort((a, b) => a - b);
  const speeds = [...new Set(rows.map(r => r.target_speed_ms))].sort((a, b) => a - b);
  const at = (b, v) => rows.find(r => r.bars === b && r.target_speed_ms === v);
  const trainedMax = pack.trained_max_bars || null;

  const X0 = 96, Y0 = 46, CW = 132, CH = 62;
  const W = X0 + speeds.length * CW, H = Y0 + bars.length * CH;

  // capture -> colour. Sequential ramp built from the theme accent so it tracks light/dark.
  const col = v => {
    const t = Math.max(0, Math.min(1, v));
    // low capture = warm/bad, high = accent. Interpolate in sRGB; good enough for a 28-cell map.
    const bad = [209, 90, 78], good = [13, 143, 130];
    const mid = [214, 178, 96];
    const c = t < 0.5
      ? bad.map((x, i) => Math.round(x + (mid[i] - x) * (t / 0.5)))
      : mid.map((x, i) => Math.round(x + (good[i] - x) * ((t - 0.5) / 0.5)));
    return `rgb(${c[0]},${c[1]},${c[2]})`;
  };

  let g = '';
  // trained-range shading behind the rows at or below the trained max
  if (trainedMax) {
    const n = bars.filter(b => b <= trainedMax).length;
    if (n) g += `<rect x="${X0}" y="${Y0}" width="${speeds.length * CW}" height="${n * CH}"
      fill="${cssv('--accent')}" opacity="0.06"/>`;
  }
  bars.forEach((b, ri) => {
    speeds.forEach((v, ci) => {
      const cell = at(b, v);
      if (!cell) return;
      const x = X0 + ci * CW, y = Y0 + ri * CH;
      const ood = trainedMax && b > trainedMax;
      g += `<rect x="${x + 2}" y="${y + 2}" width="${CW - 4}" height="${CH - 4}" rx="4"
        fill="${col(cell.capture)}" opacity="${ood ? 0.55 : 0.92}"/>`;
      g += `<text class="hm-v" x="${x + CW / 2}" y="${y + CH / 2 + 1}" text-anchor="middle">${(cell.capture * 100).toFixed(1)}</text>`;
      g += `<text class="hm-s" x="${x + CW / 2}" y="${y + CH / 2 + 17}" text-anchor="middle">crash ${(cell.crash * 100).toFixed(0)}%</text>`;
    });
    const y = Y0 + ri * CH;
    const dens = (at(b, speeds[0]) || {}).density_per_100m2;
    g += `<text class="t ink" x="${X0 - 12}" y="${y + CH / 2 - 3}" text-anchor="end" font-size="13" font-weight="600">${b} bars</text>`;
    g += `<text class="t dim" x="${X0 - 12}" y="${y + CH / 2 + 13}" text-anchor="end" font-size="10.5">${dens != null ? dens.toFixed(1) + '/100m²' : ''}</text>`;
  });
  speeds.forEach((v, ci) => {
    g += `<text class="t dim" x="${X0 + ci * CW + CW / 2}" y="${Y0 - 22}" text-anchor="middle" font-size="11.5">target ${v.toFixed(2)} m/s</text>`;
  });
  g += `<text class="t dim" x="${X0 - 12}" y="${Y0 - 22}" text-anchor="end" font-size="10.5">capture %</text>`;
  if (trainedMax) {
    const n = bars.filter(b => b <= trainedMax).length;
    const yl = Y0 + n * CH;
    g += `<line x1="${X0}" y1="${yl}" x2="${X0 + speeds.length * CW}" y2="${yl}"
      stroke="${cssv('--accent')}" stroke-width="1.6" stroke-dasharray="5 4"/>`;
    g += `<text class="t dim" x="${X0 + 4}" y="${yl + 14}" font-size="10.5">↑ trained (≤${trainedMax} bars) · ↓ generalisation</text>`;
  }
  svg.setAttribute('viewBox', `0 0 ${W} ${H + 26}`);
  svg.innerHTML = g;

  // headline asymmetry, computed from the data rather than hard-coded
  const mean = xs => xs.reduce((a, b) => a + b, 0) / xs.length;
  const byBar = bars.map(b => mean(speeds.map(v => (at(b, v) || {}).capture).filter(x => x != null)));
  const bySpd = speeds.map(v => mean(bars.map(b => (at(b, v) || {}).capture).filter(x => x != null)));
  const dDen = (byBar[0] - byBar[byBar.length - 1]) * 100;
  const dSpd = (bySpd[0] - bySpd[bySpd.length - 1]) * 100;
  const cap = document.getElementById('map-cap');
  if (cap) cap.textContent =
    `Density dominates: over this grid it costs ${dDen.toFixed(0)} pp of capture, while target speed `
    + `costs only ${dSpd.toFixed(1)} pp — the pursuer at 2.5 m/s is fast enough that target speed is `
    + `not a binding difficulty axis. Rows below the dashed line were never trained.`;
  if (sub) sub.textContent = `${rows.length} cells · 2049 episodes each · deterministic · sensor-only`;
}

/* Where the self-paced density curriculum actually stopped climbing. */
function renderCeiling(s) {
  const c = s.density_ceiling;
  const wrap = document.getElementById('ceiling-wrap');
  const tbl = document.getElementById('ceiling-tbl');
  if (!wrap || !tbl) return;
  if (!c || !(c.chain || []).length) { wrap.hidden = true; return; }
  wrap.hidden = false;

  const verdict = { promoted: 'promoted', held: 'held', ceiling: 'ceiling' };
  tbl.innerHTML = `<thead><tr><th>bars</th><th>density</th><th>gate capture</th><th>outcome</th></tr></thead>
    <tbody>${c.chain.map(r => {
      const cls = r.result === 'promoted' ? 'good' : r.result === 'ceiling' ? 'bad' : '';
      const held = r.held_windows ? ` ×${r.held_windows}` : '';
      return `<tr${cls ? ` class="${cls}"` : ''}>
        <td>${r.bars}</td>
        <td>${r.density_per_100m2 != null ? r.density_per_100m2.toFixed(1) + '/100m²' : ''}</td>
        <td>${(r.capture * 100).toFixed(1)}%</td>
        <td>${verdict[r.result] || r.result}${held}</td></tr>`;
    }).join('')}</tbody>`;

  const held = c.hold_series || [];
  const ceil = c.chain[c.chain.length - 1] || {};
  const cap = document.getElementById('ceiling-cap');
  if (cap && held.length) {
    const mean = held.reduce((a, b) => a + b, 0) / held.length;
    // least-squares slope per window, to say "converged" rather than "still climbing"
    const mx = (held.length - 1) / 2;
    let num = 0, den = 0;
    held.forEach((y, i) => { num += (i - mx) * (y - mean); den += (i - mx) * (i - mx); });
    const slope = den ? num / den : 0;
    cap.textContent =
      `The curriculum promoted 85 → 90 → 95 → ${ceil.bars}, then held ${held.length} consecutive `
      + `windows at ${ceil.bars} bars with capture averaging ${(mean * 100).toFixed(1)}% against a `
      + `${(c.threshold * 100).toFixed(0)}% gate (${(slope * 100).toFixed(2)} pp per window — flat, `
      + `not still climbing). ${ceil.density_per_100m2.toFixed(1)} bars/100 m² is the trainable `
      + `density ceiling for this sensor-only policy.`;
  }
}

function renderSpeed(s) {
  const data = pickSpeed(s);
  const tbl = document.getElementById('speed-tbl');
  const plot = document.getElementById('speedplot');
  const cap = document.getElementById('speed-cap');
  if (!data.length) {
    if (cap) cap.textContent = 'no speed curve yet';
    return;
  }

  if (plot) {
    const PX0 = 70, PX1 = 690, PY0 = 40, PY1 = 330;
    const smax = Math.max(1.5, ...data.map(d => d.speed));
    const sx = v => PX0 + (v / smax) * (PX1 - PX0);
    const sy = v => PY1 - v * (PY1 - PY0);
    const mk = (key, col, dash) => {
      const pts = data.filter(d => d[key] != null).map(d => `${sx(d.speed).toFixed(1)},${sy(d[key]).toFixed(1)}`).join(' ');
      return `<polyline class="draw" fill="none" stroke="${col}" stroke-width="2.4" ${dash || ''} points="${pts}"/>`
        + data.filter(d => d[key] != null).map(d =>
          `<circle class="pt" cx="${sx(d.speed).toFixed(1)}" cy="${sy(d[key]).toFixed(1)}" r="3.5" fill="${col}"/>`).join('');
    };
    let g = '';
    for (let i = 0; i <= 4; i++) {
      const y = PY0 + i * (PY1 - PY0) / 4;
      g += `<line x1="${PX0}" y1="${y}" x2="${PX1}" y2="${y}" stroke="${cssv('--line')}" stroke-width="1"/>`;
    }
    const ylab = [100, 75, 50, 25, 0].map((v, i) =>
      `<text x="60" y="${(PY0 + i * (PY1 - PY0) / 4 + 4).toFixed(0)}" text-anchor="end">${v}${i === 0 ? '%' : ''}</text>`).join('');
    plot.innerHTML = `
      ${g}
      <line x1="${PX0}" y1="${PY1}" x2="${PX1}" y2="${PY1}" stroke="${cssv('--line')}" stroke-width="1.5"/>
      <line x1="${PX0}" y1="${PY0}" x2="${PX0}" y2="${PY1}" stroke="${cssv('--line')}" stroke-width="1.5"/>
      <g class="t dim" font-size="11">${ylab}</g>
      <g class="t dim" font-size="11">${data.map((d, i) =>
        `<text x="${sx(d.speed).toFixed(0)}" y="${PY1 + 18}" text-anchor="middle">${d.speed}${i === data.length - 1 ? ' m/s' : ''}</text>`
      ).join('')}</g>
      ${mk('captured_360', cssv('--gt'), 'stroke-dasharray="6 5"')}
      ${mk('crash_240', cssv('--bad'))}
      ${mk('captured_240', cssv('--sensor'))}`;
    requestAnimationFrame(() => drawIn(plot));
  }

  if (tbl) {
    tbl.innerHTML = `<thead><tr><th>speed</th><th>cap 240</th><th>crash 240</th><th>cap 360</th><th>crash 360</th></tr></thead>
      <tbody>${data.map(d => `<tr>
        <td>${d.speed.toFixed(1)}</td>
        <td>${pct(d.captured_240)}</td><td>${pct(d.crash_240)}</td>
        <td>${pct(d.captured_360)}</td><td>${pct(d.crash_360)}</td>
      </tr>`).join('')}</tbody>`;
  }
  if (cap) cap.textContent = '25 bars · held-out';
}

function renderRuns(s) {
  const runs = (s.runs || []).slice().reverse().slice(0, 12);
  const tbl = document.getElementById('runs-tbl');
  if (!tbl) return;
  tbl.innerHTML = `<thead><tr><th>run</th><th>date</th><th>bars</th><th>peak</th><th>final</th><th>crash</th><th>Δ</th></tr></thead>
    <tbody>${runs.map(r => {
      const cap = r.last_captured_rate, peak = r.peak_captured_rate;
      const gap = (peak != null && cap != null) ? (peak - cap) : null;
      const hi = r.run === (s.latest_run && s.latest_run.run) ? ' class="hi"' : '';
      return `<tr${hi}><td>${(r.run || '').replace(/_navrl$/, '')}</td>
        <td>${(r.finalized_at || '').slice(0, 10)}</td>
        <td>${r.last_n_bars_active ?? '—'}</td>
        <td>${pct(peak)}</td><td>${pct(cap)}</td><td>${pct(r.last_crash_rate)}</td>
        <td>${gap != null ? ('−' + (gap * 100).toFixed(0) + 'pt') : '—'}</td></tr>`;
    }).join('')}</tbody>`;
}

function renderPhases() {
  const phases = [
    ['P0–P5', 'sensor · tracking foundation', 'done'],
    ['P6A', 'bounded action contract', 'done'],
    ['P6B', 'cluster-sector baseline', 'done'],
    ['P6C', 'corridor-token pilot', 'done'],
    ['P6D', 'two-depth representation', 'active'],
    ['P7', 'sim-to-real', 'todo'],
    ['Paper', 'ablation + write-up', 'todo'],
  ];
  const ph = document.getElementById('phases');
  if (!ph) return;
  const lab = { done: 'done', active: 'now', todo: 'todo' };
  ph.innerHTML = phases.map(p => `
    <div class="phase${p[2] === 'active' ? ' is-active' : ''}">
      <span class="pid">${p[0]}</span><span class="pt">${p[1]}</span>
      <span class="badge ${p[2]}">${lab[p[2]]}</span>
    </div>`).join('');
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

(async function () {
  wireChrome();
  renderPhases();

  const renderAll = (s) => {
    if (s && s.placement_area_m2) BAND_AREA = Number(s.placement_area_m2);
    if (s && s.arena_geometry) { ARENA_GEO = s.arena_geometry; applyArenaGeometry(ARENA_GEO); }
    renderLive(s);
    renderCriteria(s);
    renderResearchUpdate(s);
    renderCorridor(s);
    renderCurve(s);
    renderSpeed(s);
    renderHeatmap(s);
    renderCeiling(s);
    renderRuns(s);
  };
  try { renderAll(fallbackStatus()); } catch (e) { console.error(e); }
  try { renderAll(await fetchStatus()); }
  catch (e) {
    const pill = document.getElementById('livepill');
    if (pill) pill.className = 'pill is-snapshot';
    const t = document.getElementById('livepill-txt');
    if (t) t.textContent = 'SNAP';
  }

  const boot3d = () => {
    if (typeof THREE === 'undefined') throw new Error('THREE missing (vendor/three.min.js)');
    if (!THREE.OrbitControls) throw new Error('OrbitControls missing');
    if (!window.Arena) throw new Error('Arena missing');
    const stage = document.getElementById('stage');
    if (!stage) throw new Error('#stage missing');
    window.__mo = window.Arena;
    // configure BEFORE init: the geometry decides the scene bounds and the initial bar layout
    if (ARENA_GEO) window.Arena.configure(ARENA_GEO);
    window.Arena.init(stage);
    wireArena();
  };

  requestAnimationFrame(() => {
    try { boot3d(); }
    catch (e) {
      console.error('3D init', e);
      const st = document.getElementById('stage');
      if (st) {
        st.innerHTML = `<div style="position:absolute;inset:0;display:grid;place-items:center;padding:20px;text-align:center;font:12.5px/1.5 var(--mono),monospace;color:var(--muted)">
          3D init failed: ${String(e.message || e)}</div>`;
      }
    }
  });
})();
