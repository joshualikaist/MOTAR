/* MOTAR status board — data panels. Arena = window.Arena from arena.js */

const pct = x => x == null ? '—' : (x * 100).toFixed(1) + '%';
const BAND_AREA = 478;
const perc100 = n => ((n / BAND_AREA) * 100).toFixed(1);
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
        ? { k: 'epoch', v: String(Math.round(A.epoch)), c: 'acc', s: '/ 500 pilot' }
        : { k: 'peak→final', v: gapPt != null ? ('−' + gapPt + 'pt') : '—', c: gapPt >= 30 ? 'bad' : gapPt >= 15 ? 'warn' : 'good', s: pct(peak) + ' peak' },
    ];
    cards.innerHTML = rows.map(c =>
      `<div class="stat ${c.c}"><div class="v">${c.v}</div><div class="k">${c.k}</div><div class="s">${c.s}</div></div>`
    ).join('');
  }
  if (capEl) {
    capEl.textContent = live
      ? `Live = last ${A.tail_epochs || 50} epochs · fixed 25-bar fresh bounded pilot.`
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

function renderResearchUpdate(s) {
  const u = s.research_update || {};
  const pilot = u.bounded_pilot || {};
  const hero = document.getElementById('update-hero');
  const sub = document.getElementById('update-sub');
  const milestones = document.getElementById('update-milestones');
  const evalTbl = document.getElementById('update-eval');
  const decision = document.getElementById('update-decision');

  if (sub) sub.textContent = u.subtitle || 'corrected observation → bounded action contract';
  if (hero) {
    const active = pilot.is_live ? '<span class="update-live">RUNNING</span>' : '<span class="update-snap">SNAPSHOT</span>';
    hero.innerHTML = `<div><span class="eyebrow">CURRENT HYPOTHESIS</span>
      <strong>${u.headline || 'Sensor fixed; action support is the active bottleneck.'}</strong>
      <p>${u.summary || ''}</p></div>
      <div class="update-run">${active}<b>${pilot.run || '—'}</b>
      <span>ep ${pilot.epoch ?? '—'} / ${pilot.max_epochs ?? 500} · ${pilot.bars ?? 25} bars</span></div>`;
  }
  if (milestones) {
    milestones.innerHTML = (u.milestones || []).map(m => `
      <article class="milestone ${m.state || ''}">
        <span>${m.label || ''}</span><b>${m.value || ''}</b><p>${m.detail || ''}</p>
      </article>`).join('');
  }
  if (evalTbl) {
    const rows = u.legacy_eval || [];
    evalTbl.innerHTML = `<thead><tr><th>target</th><th>capture</th><th>crash</th><th>bar</th><th>below</th></tr></thead>
      <tbody>${rows.map(r => `<tr><td>${Number(r.target_speed).toFixed(2)}</td>
        <td>${pct(r.capture)}</td><td>${pct(r.crash)}</td>
        <td>${pct(r.bar_contact)}</td><td>${pct(r.below)}</td></tr>`).join('')}</tbody>`;
  }
  if (decision) {
    const gates = u.gates || [];
    decision.innerHTML = gates.map(g => `<p><b>${g.label}</b> ${g.value}</p>`).join('')
      + `<p class="decision">${u.decision || ''}</p>`;
  }
}

function pickDensity(s) {
  const c = s.density_curves || {};
  const pack = c.general_repr_density_curve || c.vision_density_curve;
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
    ['P0–P5', 'firewall · detector · perception', 'done'],
    ['P6A', 'bounded action contract', 'active'],
    ['P6B', 'density 25→110', 'todo'],
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
    renderLive(s);
    renderResearchUpdate(s);
    renderCurve(s);
    renderSpeed(s);
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
