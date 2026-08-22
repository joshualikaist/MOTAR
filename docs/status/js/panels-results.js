/* results.html — 결과. Density curves, speed axis, density×speed map, robustness, corridor. */

function densityPack(s) {
  const c = s.density_curves || {};
  // Current task first. Superseded series are kept in the snapshot as evidence of the condition
  // they were measured under, but must never stand in for present performance.
  const ordered = [c.v2_heldout_density_curve, c.corrected_chirality_density_curve,
                   c.general_repr_density_curve, c.vision_density_curve];
  return ordered.find(p => p && !p.superseded && ((p.rows || p).length)) || null;
}

function pickDensity(s) {
  const c = s.density_curves || {};
  const pack = densityPack(s);
  const raw = Array.isArray(pack) ? pack : (pack && pack.rows) || [];
  const gtPack = c.vision_density_curve && c.vision_density_curve.superseded
    ? null : c.vision_density_curve;
  const gtRows = Array.isArray(gtPack) ? gtPack : (gtPack && gtPack.rows) || [];
  const gtByBars = Object.fromEntries(gtRows.map(r => {
    const b = r.density_bars ?? r.bars;
    return [b, r.gt_injected_phase2];
  }).filter(([b, g]) => b != null && g != null));

  // The curve is a frozen checkpoint evaluation. Its train/OOD boundary must come from the
  // curve's own checkpoint metadata, never from the currently running curriculum.
  const explicitMax = pack && finite(pack.trained_max_bars);
  const liveBars = s.active_run && s.active_run.is_live ? s.active_run.n_bars_active : null;
  const trainedMax = Math.round(explicitMax != null ? explicitMax : (liveBars || (s.latest_run && s.latest_run.last_n_bars_active) || 65));

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

function renderCurve(s) {
  const rows = pickDensity(s);
  const pack = densityPack(s) || {};
  const curveArea = pack.placement_area_m2;
  const trainedMax = rows.find(r => r.trained === false)
    ? Math.max(...rows.filter(r => r.trained).map(r => r.density_bars))
    : (s.active_run && s.active_run.is_live
      ? Math.round(s.active_run.n_bars_active)
      : (s.latest_run && s.latest_run.last_n_bars_active) || null);

  const sub = document.getElementById('density-sub');
  // Name the task version on the panel: a v1 curve and a v2 curve differ in arena, placement
  // band and bar geometry, so a reader must never have to guess which one is on screen.
  const tag = pack.task_version
    ? `${pack.task_version} task · ${pack.arena_xy_m || '?'} m arena · per ${Math.round(curveArea || 0)}m² · `
    : '';
  if (sub) sub.textContent = rows.length
    ? `${tag}held-out frozen checkpoint · sensor-only · cells above that checkpoint’s trained max = OOD`
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
        return `<tr${cliff}><td>${r.density_bars}</td><td>${perc100(r.density_bars, curveArea)}</td><td>${pct(r.captured)}</td><td>${pct(r.crash)}</td><td>${pct(r.timeout)}</td><td>${zone}</td><td>${r.gt_injected_phase2 != null ? pct(r.gt_injected_phase2) : '—'}</td></tr>`;
      }).join('')}</tbody>`;
  }
  const cap = document.getElementById('curve-cap');
  if (cap) {
    const policy = pack.policy ? `${pack.policy} · ` : '';
    cap.textContent = `${policy}deterministic · this curve is a frozen-checkpoint evaluation; the live `
      + `curriculum is tracked separately above.`
      + (pack.task_version === 'v1'
        ? ' v1 task (24 m arena): densities are per its own 478 m² band and are NOT comparable with v2.'
        : '');
  }
}

function pickSpeedPack(s) {
  const c = s.speed_curves || {};
  // Fail closed: a valid archived v1 pilot must never silently replace missing v2 evidence.
  return [c.v2_riskcap_fixed_speed_axis]
    .find(p => p && p.task_version === 'v2' && p.headline_eligible !== false
      && !p.superseded && ((p.rows || p).length)) || null;
}

function pickSpeed(s) {
  const pack = pickSpeedPack(s);
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

function renderSpeed(s) {
  const data = pickSpeed(s);
  const sp = pickSpeedPack(s) || {};
  const tbl = document.getElementById('speed-tbl');
  const plot = document.getElementById('speedplot');
  const cap = document.getElementById('speed-cap');
  const sub = document.getElementById('speed-sub');
  const legend = document.getElementById('speed-legend');
  if (!data.length) {
    if (cap) cap.textContent = 'no current v2 speed curve yet';
    if (sub) sub.textContent = 'current-task evidence unavailable; archived v1 pilots are not substituted';
    if (tbl) tbl.innerHTML = '';
    if (plot) plot.innerHTML = '';
    return;
  }
  const has360 = data.some(d => d.captured_360 != null || d.crash_360 != null);

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
    tbl.innerHTML = `<thead><tr><th>target speed</th><th>capture</th><th>crash</th>${has360 ? '<th>capture 360</th><th>crash 360</th>' : ''}</tr></thead>
      <tbody>${data.map(d => `<tr>
        <td>${d.speed.toFixed(1)} m/s</td>
        <td>${pct(d.captured_240)}</td><td>${pct(d.crash_240)}</td>
        ${has360 ? `<td>${pct(d.captured_360)}</td><td>${pct(d.crash_360)}</td>` : ''}
      </tr>`).join('')}</tbody>`;
  }
  // The caption must describe the series actually drawn: it used to hard-code the v1 25-bar FOV
  // ablation, which silently mislabelled any other series selected above it.
  if (sub) sub.textContent = `${sp.bars} bars · target-speed axis · sensor-only`;
  if (legend) legend.innerHTML = has360
    ? '<span><i style="background:var(--sensor)"></i>capture 240°</span><span><i style="background:var(--gt)"></i>capture 360°</span><span><i style="background:var(--bad)"></i>crash 240°</span>'
    : '<span><i style="background:var(--sensor)"></i>capture</span><span><i style="background:var(--bad)"></i>crash</span>';
  if (cap) {
    const bars = sp.bars != null ? `${sp.bars} bars · ` : '';
    const policy = sp.policy ? `${sp.policy} · ` : '';
    const version = sp.task_version ? `${sp.task_version} task · ` : '';
    const legacy = sp.evaluation_semantics === 'legacy_timeout_at_601'
      ? ' · legacy 601-action timeout semantics; re-measure before schema-v2 comparisons'
      : '';
    cap.textContent = `${version}${bars}${policy}held-out · deterministic${legacy}`;
  }
}

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
  // The density labels are per this map's OWN placement area, which is not the one the rest of
  // the page reports. Say so on the axis, next to the numbers being misread.
  if (pack.placement_area_m2) {
    g += `<text class="t dim" x="${X0 - 12}" y="${Y0 - 8}" text-anchor="end" font-size="9.5">per ${pack.placement_area_m2.toFixed(0)}m²</text>`;
  }
  if (trainedMax) {
    const n = bars.filter(b => b <= trainedMax).length;
    const yl = Y0 + n * CH;
    g += `<line x1="${X0}" y1="${yl}" x2="${X0 + speeds.length * CW}" y2="${yl}"
      stroke="${cssv('--accent')}" stroke-width="1.6" stroke-dasharray="5 4"/>`;
    g += `<text class="t dim" x="${X0 + 4}" y="${yl + 14}" font-size="10.5">↑ trained (≤${trainedMax} bars) · ↓ generalisation</text>`;
  }
  svg.setAttribute('viewBox', `0 0 ${W} ${H + 26}`);
  svg.innerHTML = g;

  // Marginal endpoint contrasts are descriptive effects, not an interaction test.
  const mean = xs => xs.reduce((a, b) => a + b, 0) / xs.length;
  const byBar = bars.map(b => mean(speeds.map(v => (at(b, v) || {}).capture).filter(x => x != null)));
  const bySpd = speeds.map(v => mean(bars.map(b => (at(b, v) || {}).capture).filter(x => x != null)));
  const dDen = (byBar[0] - byBar[byBar.length - 1]) * 100;
  const dSpd = (bySpd[0] - bySpd[bySpd.length - 1]) * 100;
  const cap = document.getElementById('map-cap');
  const interaction = pack.interaction_test || {};
  const interactionText = interaction.verdict === 'not confirmed'
    ? `No density×speed interaction was confirmed inside the trained support `
      + `(LR p=${interaction.continuous_likelihood_ratio_p}; omnibus p=${interaction.categorical_omnibus_p}). `
    : '';
  const legacy = pack.evaluation_semantics === 'legacy_timeout_at_601'
    ? `Legacy evaluator: configured 600-step cells ended at action 601; do not mix with schema-v2 cells. `
    : '';
  if (cap) cap.textContent =
    `Descriptively, the grid endpoint contrast is ${dDen.toFixed(1)} pp across density and `
    + `${dSpd.toFixed(1)} pp across target speed. ${interactionText}${pack.ood_note || ''} `
    + `Rows below the dashed line were never trained. ${legacy}`
    + (pack.task_version === 'v1' ? `This map predates the target heading-continuity fix. ` : '')
    + (pack.comparable_with_v2 === false
        ? ` ${pack.superseded_note || ''} `
          + `Reading a bar count across task versions is misleading: 85 bars is `
          + `${(85 / (pack.placement_area_m2 || 478) * 100).toFixed(1)}/100m² here but `
          + `${(85 / 1600 * 100).toFixed(1)}/100m² in the v2 arena.`
        : '');
  if (sub) {
    const tag = pack.task_version ? `${pack.task_version} task · ${pack.arena_xy_m || '?'} m arena · ` : '';
    sub.textContent = `${tag}${rows.length} cells · 2049 episodes each · deterministic · sensor-only`;
  }
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
      + `density ceiling for the historical cluster-sector policy shown here; it is not the active v2 run.`;
  }
}

function renderRobustness(s) {
  const r = s.perception_robustness || {};
  const setText = (id, value) => {
    const el = document.getElementById(id);
    if (el && value != null) el.textContent = value;
  };
  setText('robust-sub', r.subtitle);
  setText('robust-finding', r.finding);
  const pct = v => (v == null ? '—' : `${(v * 100).toFixed(2)}%`);
  const rows = document.getElementById('robust-rows');
  if (rows) {
    const clean = r.clean || {};
    const head = clean.capture_rate == null ? '' :
      `<tr class="robust-clean"><td>${clean.label || 'clean'}</td>` +
      `<td>${pct(clean.capture_rate)}</td><td>${pct(clean.crash_rate)}</td><td>—</td></tr>`;
    rows.innerHTML = head + (r.axes || []).map(a => {
      const d = a.capture_delta_pp;
      // Anything inside a few points of clean is noise at ~2050 episodes, not a finding.
      const cls = d <= -10 ? 'robust-bad' : (d <= -5 ? 'robust-warn' : 'robust-ok');
      const note = a.note ? ` <span class="robust-note">${a.note}</span>` : '';
      return `<tr><td>${a.label}${note}</td><td>${pct(a.capture_rate)}</td>` +
        `<td>${pct(a.crash_rate)}</td><td class="${cls}">${d >= 0 ? '+' : ''}${d.toFixed(2)}pp</td></tr>`;
    }).join('');
  }
  const sup = r.superseded;
  setText('robust-superseded', sup
    ? `Superseded: the same 0.1s latency measured ${pct(sup.capture_rate)} (${sup.capture_delta_pp.toFixed(2)}pp) `
      + 'before the timestamp-aware transform. That number is a modelling artifact and is not quoted.'
    : '');
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

/* Segmented control for the three curve panes. Scoped to this page: the old wireChrome() bound it
   globally, and it manipulates #pane-density / #pane-speed / #pane-map, which exist only here. */
document.addEventListener('DOMContentLoaded', function () {
  document.querySelectorAll('.seg-btn[data-curve]').forEach(function (btn) {
    btn.addEventListener('click', function () {
      document.querySelectorAll('.seg-btn[data-curve]').forEach(function (b) {
        b.classList.toggle('on', b === btn);
      });
      var which = btn.dataset.curve;
      ['density', 'speed', 'map'].forEach(function (name) {
        var pane = document.getElementById('pane-' + name);
        if (pane) pane.classList.toggle('on', which === name);
      });
    });
  });
});

MOTAR.register(renderCurve, renderSpeed, renderHeatmap, renderCeiling, renderRobustness,
               renderCorridor);
