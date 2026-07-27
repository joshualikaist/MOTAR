/* MOTAR research-status — data panels + Motionsites/Emil interaction chrome.
   3D arena lives in arena.js (window.Arena). This file never redefines it. */

const pct = x => x == null ? '—' : (x * 100).toFixed(1) + '%';
const BAND_AREA = 478;
const perc100 = n => ((n / BAND_AREA) * 100).toFixed(1);
const cssv = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
const finePointer = matchMedia('(hover: hover) and (pointer: fine)').matches;

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

/* ---------------- Motionsites / Emil chrome ---------------- */
function wireChrome() {
  const rail = document.getElementById('railtop');
  const onScroll = () => {
    if (!rail) return;
    const max = document.documentElement.scrollHeight - innerHeight;
    const t = max > 0 ? scrollY / max : 0;
    rail.style.transform = `scaleX(${Math.min(1, Math.max(0, t))})`;
  };
  addEventListener('scroll', onScroll, { passive: true });
  onScroll();

  // scroll reveals — fire once, then drop will-change (Emil: don't keep layers alive)
  if (!reduceMotion) {
    const io = new IntersectionObserver((entries) => {
      entries.forEach(e => {
        if (!e.isIntersecting) return;
        e.target.classList.add('in');
        setTimeout(() => e.target.classList.add('done'), 700);
        io.unobserve(e.target);
      });
    }, { threshold: 0.12, rootMargin: '0px 0px -8% 0px' });
    document.querySelectorAll('.reveal').forEach(el => io.observe(el));
  } else {
    document.querySelectorAll('.reveal').forEach(el => el.classList.add('in', 'done'));
  }

  // magnetic buttons — decorative, fine pointer only (Emil: skip on frequent/coarse)
  if (finePointer && !reduceMotion) {
    document.querySelectorAll('.magnetic').forEach(btn => {
      btn.addEventListener('pointermove', e => {
        const r = btn.getBoundingClientRect();
        const x = (e.clientX - r.left) / r.width - 0.5;
        const y = (e.clientY - r.top) / r.height - 0.5;
        btn.style.setProperty('--mx', (x * 6).toFixed(2) + 'px');
        btn.style.setProperty('--my', (y * 6).toFixed(2) + 'px');
      });
      btn.addEventListener('pointerleave', () => {
        btn.style.setProperty('--mx', '0px');
        btn.style.setProperty('--my', '0px');
      });
    });

    // card tilt — Motionsites 3D hover, spring-back on leave
    document.querySelectorAll('.tilt').forEach(card => {
      card.addEventListener('pointermove', e => {
        const r = card.getBoundingClientRect();
        const x = e.clientX - r.left, y = e.clientY - r.top;
        const rx = -((y - r.height / 2) / 18);
        const ry = (x - r.width / 2) / 18;
        card.style.setProperty('--rx', rx.toFixed(2) + 'deg');
        card.style.setProperty('--ry', ry.toFixed(2) + 'deg');
      });
      card.addEventListener('pointerleave', () => {
        card.style.setProperty('--rx', '0deg');
        card.style.setProperty('--ry', '0deg');
      });
    });
  }

  const theme = document.getElementById('themebtn');
  if (theme) theme.addEventListener('click', toggleTheme);
}

/* ---------------- live / hero ---------------- */
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
  if (pillTxt) pillTxt.textContent = live ? '학습 진행 중' : '최신 완료 run';
  if (pillRun) pillRun.textContent = runName;

  const gen = (s.generated_at || '').slice(0, 10);
  const footGen = document.getElementById('foot-gen');
  const footRuns = document.getElementById('foot-runs');
  if (footGen) footGen.textContent = gen || '—';
  if (footRuns) footRuns.textContent = s.n_runs || (s.runs ? s.runs.length : '—');

  const h2 = document.getElementById('live-h2');
  const sub = document.getElementById('live-sub');
  const cards = document.getElementById('live-cards');
  const capEl = document.getElementById('live-cap');
  if (h2) h2.textContent = live ? '지금 이 순간 학습 중' : '가장 최근 완료된 학습';
  if (sub) {
    sub.innerHTML = live
      ? `<b>${runName}</b> · epoch ${Math.round(A.epoch)} · 활성 막대 <b>${Math.round(A.n_bars_active)}</b> · 목표거리 ${Number(A.curriculum_max_m).toFixed(1)} m · 최근 ${A.tail_epochs || 50} epoch 평균`
      : `<b>${runName}</b> · ${L.epochs_logged || '?'} epoch · 활성 막대 ${L.last_n_bars_active ?? '?'} · 목표거리 ${L.last_curriculum_max_m ?? '?'} m`;
  }

  const peak = L.peak_captured_rate;
  const finalCap = live ? A.captured_rate : L.last_captured_rate;
  const finalCrash = live ? A.crash_rate : L.last_crash_rate;
  const finalTo = live ? A.timeout_rate : L.last_timeout_rate;
  const gapPt = (peak != null && L.last_captured_rate != null)
    ? ((peak - L.last_captured_rate) * 100).toFixed(0) : null;

  if (cards) {
    const rows = [
      { k: live ? '포획률 (live)' : '포획률 (최종)', v: pct(finalCap), c: finalCap >= 0.7 ? 'good' : finalCap >= 0.55 ? 'warn' : 'bad', s: live ? '최근 창 평균' : '학습 종료 시점' },
      { k: '충돌률', v: pct(finalCrash), c: finalCrash <= 0.15 ? 'good' : finalCrash <= 0.35 ? 'warn' : 'bad', s: '주 실패 모드' },
      { k: 'timeout', v: pct(finalTo), c: 'acc', s: '시간 제한 종료' },
      { k: '포획률 (peak)', v: pct(peak), c: 'acc', s: 'ep ' + (L.peak_captured_epoch || '?') },
      { k: 'peak→final', v: gapPt != null ? ('−' + gapPt + 'pt') : '—', c: gapPt >= 30 ? 'bad' : gapPt >= 15 ? 'warn' : 'good', s: '과특화 지표' },
    ];
    cards.innerHTML = rows.map(c =>
      `<div class="stat ${c.c}"><div class="v">${c.v}</div><div class="k">${c.k}</div><div class="s">${c.s}</div></div>`).join('');
  }
  if (capEl) {
    capEl.innerHTML = live
      ? `라이브 수치는 최근 ${A.tail_epochs || 50} epoch 평균입니다. 밀도 커리큘럼이 진행 중이면 승급 직후 포획률이 잠시 내려갔다가 회복하는 것이 정상입니다.`
      : `peak ${pct(peak)}(ep ${L.peak_captured_epoch}) → final ${pct(L.last_captured_rate)}. 밀도·거리 커리큘럼이 끝에서 겹치면 peak→final 격차가 커집니다.`;
  }

  // sync arena preset button to live density
  const preset = document.getElementById('btn-preset');
  const nBars = live ? Math.round(A.n_bars_active) : (L.last_n_bars_active || 70);
  if (preset) preset.textContent = `학습 도달 밀도 · ${nBars}`;
}

/* ---------------- density curve ---------------- */
function pickDensity(s) {
  const c = s.density_curves || {};
  // Prefer current FOV-240 representation curve; fall back to older vision curve.
  const pack = c.general_repr_density_curve || c.vision_density_curve;
  const raw = Array.isArray(pack) ? pack : (pack && pack.rows) || [];
  const gtPack = c.vision_density_curve;
  const gtRows = Array.isArray(gtPack) ? gtPack : (gtPack && gtPack.rows) || [];
  const gtByBars = Object.fromEntries(gtRows.map(r => {
    const b = r.density_bars ?? r.bars;
    return [b, r.gt_injected_phase2];
  }).filter(([b, g]) => b != null && g != null));

  // Trained ceiling: active live bars, else latest_run, else notes hint (~65).
  const liveBars = s.active_run && s.active_run.is_live ? s.active_run.n_bars_active : null;
  const trainedMax = Math.round(
    liveBars
    || (s.latest_run && s.latest_run.last_n_bars_active)
    || 65
  );

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
  const trainedMax = rows.find(r => r.trained === false)
    ? Math.max(...rows.filter(r => r.trained).map(r => r.density_bars))
    : (s.active_run && s.active_run.is_live
      ? Math.round(s.active_run.n_bars_active)
      : (s.latest_run && s.latest_run.last_n_bars_active) || null);

  const sub = document.getElementById('density-sub');
  if (sub) {
    sub.innerHTML = rows.length
      ? `단일 정책의 held-out 평가입니다. 학습 범위 밖의 셀은 일반화이며, 방법의 천장으로 읽으면 안 됩니다.`
      : '밀도 곡선 데이터가 아직 없습니다.';
  }
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
    g += `<line x1="${PX0}" y1="${y}" x2="${PX1}" y2="${y}" stroke="${cssv('--line-2')}" stroke-width="1"/>`;
  }
  const trainedBand = trainedMax != null && trainedMax >= xmin && trainedMax <= xmax ? `
    <rect x="${PX0}" y="${PY0}" width="${(sx(trainedMax) - PX0).toFixed(1)}" height="${PY1 - PY0}"
      fill="${cssv('--accent')}" opacity="0.07"/>
    <line x1="${sx(trainedMax).toFixed(1)}" y1="${PY0}" x2="${sx(trainedMax).toFixed(1)}" y2="${PY1}"
      stroke="${cssv('--accent')}" stroke-width="1.5" stroke-dasharray="4 4" opacity="0.75"/>
    <text class="t dim" x="${(sx(trainedMax) + 7).toFixed(1)}" y="${PY0 + 14}" font-size="10.5">
      ← 학습 범위 (${trainedMax}막대)</text>` : '';
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
      <text class="t dim" x="380" y="395" font-size="12" text-anchor="middle">활성 막대 수 →</text>
      ${trainedBand}
      <polyline class="draw" fill="none" stroke="${cssv('--gt')}" stroke-width="2.4" stroke-dasharray="6 5" points="${path('gt_injected_phase2')}"/>
      ${dots('gt_injected_phase2', cssv('--gt'), 4)}
      <polyline class="draw" fill="none" stroke="${cssv('--bad')}" stroke-width="2.2" points="${path('crash')}"/>
      ${dots('crash', cssv('--bad'), 3.5)}
      <polyline class="draw" fill="none" stroke="${cssv('--sensor')}" stroke-width="3" points="${path('captured')}"/>
      ${dots('captured', cssv('--sensor'), 4.5)}`;
    requestAnimationFrame(() => drawIn(curve));
  }

  const head = `<thead><tr><th>막대</th><th>밀도/100m²</th><th>포획(sensor)</th><th>충돌</th><th>timeout</th><th>학습범위</th><th>포획(GT)</th></tr></thead>`;
  const body = rows.map(r => {
    const cliff = r.crash >= 0.5 ? ' class="cliff"' : '';
    const zone = r.trained === false
      ? '<span class="tag-ood">범위 밖</span>'
      : (r.trained ? '<span class="tag-tr">학습됨</span>' : '—');
    return `<tr${cliff}><td>${r.density_bars}</td><td>${perc100(r.density_bars)}</td><td>${pct(r.captured)}</td><td>${pct(r.crash)}</td><td>${pct(r.timeout)}</td><td>${zone}</td><td>${r.gt_injected_phase2 != null ? pct(r.gt_injected_phase2) : '—'}</td></tr>`;
  }).join('');
  const tbl = document.getElementById('curve-tbl');
  if (tbl) tbl.innerHTML = head + '<tbody>' + body + '</tbody>';
  const cap = document.getElementById('curve-cap');
  if (cap) cap.textContent = 'held-out · deterministic · sensor-only actor · FOV 240°';
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
  setTimeout(() => svg.querySelectorAll('.pt').forEach(el => el.classList.add('in')), 200);
}

/* ---------------- speed curve ---------------- */
function pickSpeed(s) {
  const c = s.speed_curves || {};
  const pack = c.general_repr_fov240_speed_axis || c.general_repr_speed_axis;
  const raw = Array.isArray(pack) ? pack : (pack && pack.rows) || [];
  return raw.map(r => {
    const speed = r.speed ?? r.target_speed ?? r.target_speed_ms;
    // FOV ablation CSV uses capture_fov240 / capture_fov360_baseline;
    // single-curve CSVs use capture / crash.
    const c240 = r.capture_fov240 ?? r.captured_240 ?? r.captured ?? r.capture;
    const c360 = r.capture_fov360_baseline ?? r.captured_360;
    const crash240 = r.bar_contact_fov240 ?? r.crash_240 ?? r.crash;
    const crash360 = r.bar_contact_fov360_baseline ?? r.crash_360;
    return {
      speed,
      captured_240: c240,
      crash_240: crash240,
      captured_360: c360,
      crash_360: crash360,
    };
  }).filter(d => d.speed != null).sort((a, b) => a.speed - b.speed);
}

function renderSpeed(s) {
  const data = pickSpeed(s);
  const tbl = document.getElementById('speed-tbl');
  const plot = document.getElementById('speedplot');
  const cap = document.getElementById('speed-cap');
  if (!data.length) {
    if (cap) cap.textContent = '속도 곡선 데이터가 아직 없습니다.';
    return;
  }

  if (plot) {
    const PX0 = 70, PX1 = 690, PY0 = 40, PY1 = 330;
    const smax = Math.max(1.5, ...data.map(d => d.speed));
    const sx = v => PX0 + (v / smax) * (PX1 - PX0);
    const sy = v => PY1 - v * (PY1 - PY0);
    const mk = (key, col, dash) => {
      const pts = data.filter(d => d[key] != null).map(d => `${sx(d.speed).toFixed(1)},${sy(d[key]).toFixed(1)}`).join(' ');
      return `<polyline class="draw" fill="none" stroke="${col}" stroke-width="2.6" ${dash || ''} points="${pts}"/>`
        + data.filter(d => d[key] != null).map(d =>
          `<circle class="pt" cx="${sx(d.speed).toFixed(1)}" cy="${sy(d[key]).toFixed(1)}" r="4" fill="${col}"/>`).join('');
    };
    let g = '';
    for (let i = 0; i <= 4; i++) {
      const y = PY0 + i * (PY1 - PY0) / 4;
      g += `<line x1="${PX0}" y1="${y}" x2="${PX1}" y2="${y}" stroke="${cssv('--line-2') || cssv('--line2')}" stroke-width="1"/>`;
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
    tbl.innerHTML = `<thead><tr><th>속도</th><th>포획 240°</th><th>충돌 240°</th><th>포획 360°</th><th>충돌 360°</th></tr></thead>
      <tbody>${data.map(d => `<tr>
        <td>${Number(d.speed).toFixed(1)} m/s</td>
        <td>${pct(d.captured_240)}</td><td>${pct(d.crash_240)}</td>
        <td>${pct(d.captured_360)}</td><td>${pct(d.crash_360)}</td>
      </tr>`).join('')}</tbody>`;
  }
  if (cap) cap.textContent = '25막대 · held-out · FOV 240° vs 360°';
}

/* ---------------- campaign ---------------- */
function renderCampaign() {
  const el = document.getElementById('campaign');
  if (!el) return;
  const rows = [
    { lab: 'baseline · 고정 스폰', cap: 0.735, crash: null },
    { lab: '고도 PI', cap: 0.802, crash: 0.13 },
    { lab: 'tilt 보상', cap: 0.837, crash: 0.13 },
    { lab: 'look-ahead 12 m', cap: 0.851, crash: 0.127 },
    { lab: '장애물 표현 (8토큰·72빔)', cap: 0.861, crash: 0.091 },
    { lab: '토큰 FOV 240°', cap: 0.889, crash: 0.075, best: true },
  ];
  el.innerHTML = rows.map((r, i) => `
    <div class="crow${r.best ? ' best' : ''}" style="--i:${i}">
      <span class="lab">${r.lab}</span>
      <span class="track"><i class="fill" style="--f:${r.cap}"></i></span>
      <span class="num">${pct(r.cap)}</span>
      <span class="num">${r.crash != null ? pct(r.crash) : '—'}</span>
    </div>`).join('');
  // animate fills when section enters
  const io = new IntersectionObserver((entries) => {
    entries.forEach(e => {
      if (!e.isIntersecting) return;
      e.target.querySelectorAll('.fill').forEach(f => f.classList.add('in'));
      io.disconnect();
    });
  }, { threshold: 0.2 });
  io.observe(el);
}

/* ---------------- runs table ---------------- */
function renderRuns(s) {
  const runs = (s.runs || []).slice().reverse();
  const head = `<thead><tr><th>run</th><th>날짜</th><th>막대</th><th>peak</th><th>final</th><th>충돌</th><th>peak→final</th></tr></thead>`;
  const body = runs.map(r => {
    const cap = r.last_captured_rate, peak = r.peak_captured_rate;
    const gap = (peak != null && cap != null) ? (peak - cap) : null;
    const hi = r.run === (s.latest_run && s.latest_run.run) ? ' class="hi"' : '';
    return `<tr${hi}><td>${(r.run || '').replace(/_navrl$/, '')}</td>
      <td>${(r.finalized_at || '').slice(0, 10)}</td>
      <td>${r.last_n_bars_active ?? '—'}</td>
      <td>${pct(peak)}</td><td>${pct(cap)}</td><td>${pct(r.last_crash_rate)}</td>
      <td>${gap != null ? ('−' + (gap * 100).toFixed(0) + 'pt') : '—'}</td></tr>`;
  }).join('');
  const tbl = document.getElementById('runs-tbl');
  if (tbl) tbl.innerHTML = head + '<tbody>' + body + '</tbody>';
}

/* ---------------- static prose ---------------- */
function renderStatic() {
  const diagnosis = document.getElementById('diagnosis');
  if (diagnosis) {
    diagnosis.innerHTML = `
      <div class="callout hero tilt reveal" style="--i:0">
        <h3>현재 성능 · sensor-only, held-out</h3>
        <div class="kpis">
          <div class="kpi"><b>0.90</b><span>포획 · 25막대</span></div>
          <div class="kpi"><b>0.80</b><span>포획 · 50막대</span></div>
          <div class="kpi"><b>0.68</b><span>포획 · 65막대</span></div>
          <div class="kpi"><b>70</b><span>학습 도달 밀도</span></div>
        </div>
        <p>128-env · 셀당 ~2049 episode · deterministic. actor 관측에 GT 표적 없음
          (LiDAR 72×4@12 m + 전방 RGB-D 검출기만). 25막대에서 표적속도 0–1.5 m/s 전 구간 포획 0.86–0.91.</p>
      </div>
      <div class="callout tilt reveal" style="--i:1">
        <h3>측정으로 기각된 가설</h3>
        <p><b>look-ahead가 부족해서 부딪힌다</b> → 기각. 8→12 m로 늘려도 막대충돌 ~12.7% 불변.</p>
        <p><b>토큰 용량(5개)이 병목</b> → 기각. 선택 규칙(360° 최근접)이 진짜 문제였고, 240°로 좁히자 +2.7 pt.</p>
        <p><b>시간에 따라 밀도만 올리면 된다</b> → 기각. 문턱 0.55·창 4096로 몰아치자 붕괴. competence 게이트(0.70·16384)로 회복.</p>
      </div>`;
  }

  const next = document.getElementById('nextplan');
  if (next) {
    next.innerHTML = `
      <div class="callout tilt reveal" style="--i:0">
        <h3>커리큘럼 연장 · → 110막대 <span class="badge active">진행 중</span></h3>
        <p>65→70막대 구간 진행 중. 승급은 capture가 0.70을 회복했을 때만. 110막대(NavRL 앵커)까지
          competence 게이트로 밀어 올리는 것이 지금 최우선입니다.</p>
      </div>
      <div class="callout tilt reveal" style="--i:1">
        <h3>밀도 × 속도 지도</h3>
        <p>25막대에서 속도 축은 평탄합니다. 밀도 축이 확보되면 두 축을 교차해 논문 핵심 그림을 완성합니다.</p>
      </div>`;
  }

  const phases = [
    ['Baseline', 'navigation·밀도 스윕·geometry 절벽 규명', 'done'],
    ['P0', 'GT→actor 정보 방화벽', 'done'],
    ['P1–P3', 'detector · tracker · Transformer actor', 'done'],
    ['P4–P5', '이동표적 · 충돌 저감(표현/FOV)', 'done'],
    ['P6', '밀도 커리큘럼 25→110 (진행 중)', 'active'],
    ['P7', 'sim-to-real · latency · noise', 'todo'],
    ['Paper', 'oracle/perception ablation + 논문화', 'todo'],
  ];
  const ph = document.getElementById('phases');
  if (ph) {
    ph.innerHTML = phases.map(p => {
      const lab = { done: '완료', active: '진행 중', todo: '예정' }[p[2]];
      return `<div class="phase${p[2] === 'active' ? ' is-active' : ''}">
        <span class="pid">${p[0]}</span><span class="pt">${p[1]}</span>
        <span class="badge ${p[2]}">${lab}</span></div>`;
    }).join('');
  }

  // re-observe newly injected reveals
  if (!reduceMotion) {
    const io = new IntersectionObserver((entries) => {
      entries.forEach(e => {
        if (!e.isIntersecting) return;
        e.target.classList.add('in');
        setTimeout(() => e.target.classList.add('done'), 700);
        io.unobserve(e.target);
      });
    }, { threshold: 0.12 });
    document.querySelectorAll('#diagnosis .reveal, #nextplan .reveal').forEach(el => io.observe(el));
  }
}

/* ---------------- arena controls ---------------- */
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
    hudB.textContent = n;
    hudD.textContent = perc100(n);
    Arena.setBars(n);
  };
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
    play.textContent = playing ? '일시정지' : '재생';
    play.setAttribute('aria-pressed', playing ? 'true' : 'false');
  });
  document.getElementById('cb-lidar').addEventListener('change', e => Arena.setLidar(e.target.checked));
  document.getElementById('cb-camera').addEventListener('change', e => Arena.setCamera(e.target.checked));
  document.getElementById('cb-trails').addEventListener('change', e => Arena.setTrails(e.target.checked));
  document.getElementById('btn-view').addEventListener('click', () => Arena.cycleView());
  document.getElementById('btn-preset').addEventListener('click', () => {
    const m = (document.getElementById('btn-preset').textContent.match(/(\d+)/) || [])[1];
    bars.value = m || 70;
    upd();
  });
  upd();
}

/* ---------------- boot ---------------- */
(async function () {
  wireChrome();
  try { renderStatic(); renderCampaign(); } catch (e) { console.error(e); }

  const renderAll = (s) => {
    renderLive(s);
    renderCurve(s);
    renderSpeed(s);
    renderRuns(s);
  };
  try { renderAll(fallbackStatus()); } catch (e) { console.error(e); }
  try { renderAll(await fetchStatus()); }
  catch (e) {
    const pill = document.getElementById('livepill');
    if (pill) { pill.className = 'pill is-snapshot'; }
    const t = document.getElementById('livepill-txt');
    if (t) t.textContent = 'snapshot';
  }

  const boot3d = () => {
    if (typeof THREE === 'undefined' || !THREE.OrbitControls || !window.Arena)
      throw new Error('three.js / Arena unavailable');
    const stage = document.getElementById('stage');
    if (!stage) throw new Error('#stage missing');
    window.__mo = window.Arena;
    window.Arena.init(stage);
    wireArena();
    // default moving target so motion is obvious immediately
    const speed = document.getElementById('sl-speed');
    if (speed) {
      speed.value = '5';
      document.getElementById('lbl-speed').textContent = '0.5';
      window.Arena.setSpeed(0.5);
    }
  };
  try {
    // Wait one frame so .stage-frame height is resolved before WebGL sizing.
    requestAnimationFrame(() => {
      try { boot3d(); }
      catch (e) {
        console.error('3D init', e);
        const st = document.getElementById('stage');
        if (st) {
          const msg = document.createElement('div');
          msg.style.cssText = 'position:absolute;inset:0;display:grid;place-items:center;font:13px/1.5 IBM Plex Mono,monospace;color:#4d5f70;padding:24px;text-align:center;background:#d5e7f2';
          msg.textContent = '3D 뷰를 불러오지 못했습니다 (three.js CDN). 인터넷 연결 후 새로고침하세요.';
          st.appendChild(msg);
        }
        document.querySelectorAll('.dock input,.dock button').forEach(el => { el.disabled = true; });
      }
    });
  } catch (e) { console.error(e); }
})();
