/* index.html — 개요. Live metrics, success criteria, current campaign, training-run log. */

function renderLive(s) {
  const A = s.active_run;
  const L = s.latest_run || {};
  const live = A && A.is_live;
  const E = ((s.research_update || {}).active_experiment || {});
  const ref5inP2 = !live && E.ref5in_p2 === true;
  const finalAudit = !live && E.core_audit_complete === true;
  const riskcapFinal = !live && E.speed_governor_mode === 'riskcap'
    && E.post_evaluation_complete === true && E.generalization_pass === true;
  const causalAudit = finalAudit && E.causal_checks_1to3_complete === true;
  const completeAudit = finalAudit && E.causal_checks_complete === true;
  const abExperiment = E.ab_experiment === true;
  const src = live ? A : L;
  const runName = (ref5inP2 ? E.run : ((src && src.run) || '—')).replace(/_navrl$/, '');

  // The pill, #top-context, #foot-gen and #foot-runs moved to js/shell.js on 2026-08-13: they are
  // page chrome and have to be identical on all six pages. Call it here so the fetch pass updates
  // them with live data (shell.js only paints the instant-paint snapshot).
  if (window.renderShellPill) window.renderShellPill(s);

  const freshness = document.getElementById('live-freshness');

  const barsNow = live ? finite(A.n_bars_active)
    : ((finalAudit || abExperiment || riskcapFinal) ? finite(E.bars) : finite(L.last_n_bars_active));
  const tailCapture = live ? finite(A.captured_rate)
    : ((abExperiment || riskcapFinal) ? finite(E.heldout_capture)
      : (finalAudit ? finite(E.deterministic_capture) : finite(L.last_captured_rate)));
  if (freshness) {
    if (!live) {
      freshness.textContent = `snapshot · ${(s.generated_at || '').slice(0, 16).replace('T', ' ') || '—'} UTC`;
      freshness.className = 'snapshot';
    } else {
      const age = finite(A.metrics_age_min);
      freshness.textContent = age == null ? 'live · freshness unknown'
        : `live data · ${age < 1 ? '<1' : age.toFixed(0)} min old`;
      freshness.className = age != null && age > 10 ? 'stale' : 'fresh';
    }
  }

  const h2 = document.getElementById('live-h2');
  const sub = document.getElementById('live-sub');
  const cards = document.getElementById('live-cards');
  const capEl = document.getElementById('live-cap');
  if (h2) h2.textContent = live ? 'Live' : (ref5inP2 ? 'Ref5in held-out gate' : (riskcapFinal ? 'Riskcap held-out' : (causalAudit ? 'Causal held-out' : (finalAudit ? 'Final held-out' : 'Latest run'))));
  if (sub) {
    sub.textContent = ref5inP2
      ? `${runName} · frozen ep ${E.epoch} · deterministic · n=${E.deterministic_episodes}`
      : riskcapFinal
      ? `${runName} · trained winner · seed45 deterministic · n=${E.heldout_episodes}`
      : finalAudit
      ? `${runName} · frozen ep ${E.epoch} · deterministic deployment · n=${E.deterministic_episodes}`
      : live
      ? `${runName} · ep ${Math.round(A.epoch)} · bars ${Math.round(A.n_bars_active)} · goal ${Number(A.curriculum_max_m).toFixed(1)} m · tail ${A.tail_epochs || 50}`
      : `${runName} · ${L.epochs_logged || '?'} ep · bars ${L.last_n_bars_active ?? '?'} · goal ${L.last_curriculum_max_m ?? '?'} m`;
  }

  const peak = L.peak_captured_rate;
  const finalCap = live ? A.captured_rate
    : (riskcapFinal ? E.heldout_capture : (finalAudit ? E.deterministic_capture : L.last_captured_rate));
  const finalCrash = live ? A.crash_rate
    : (riskcapFinal ? E.heldout_crash : (finalAudit ? E.deterministic_crash : L.last_crash_rate));
  const finalTo = live ? A.timeout_rate
    : (riskcapFinal ? E.heldout_timeout : (finalAudit ? E.deterministic_timeout : L.last_timeout_rate));
  const gapPt = (peak != null && L.last_captured_rate != null)
    ? ((peak - L.last_captured_rate) * 100).toFixed(0) : null;
  const bars = live ? A.n_bars_active : ((finalAudit || riskcapFinal) ? E.bars : L.last_n_bars_active);

  if (cards) {
    const rows = [
      { k: 'capture', v: pct(finalCap), c: finalCap >= 0.7 ? 'good' : finalCap >= 0.55 ? 'warn' : 'bad', s: live ? 'tail avg' : ((finalAudit || riskcapFinal) ? 'held-out deploy' : 'final') },
      { k: 'crash', v: pct(finalCrash), c: finalCrash <= 0.15 ? 'good' : finalCrash <= 0.35 ? 'warn' : 'bad', s: 'fail mode' },
      { k: 'timeout', v: pct(finalTo), c: 'acc', s: '' },
      { k: 'bars', v: bars != null ? String(Math.round(bars)) : '—', c: 'acc', s: 'active' },
      ref5inP2
        ? { k: 'P2 gate', v: E.p2_verdict, c: 'bad', s: `timeout ${pct(E.deterministic_timeout)}` }
        : riskcapFinal
        ? { k: 'seed46', v: '3 / 3', c: 'good', s: 'speed gates' }
        : finalAudit
        ? { k: 'deploy−sample', v: '+' + ((E.deterministic_capture - E.stochastic_capture) * 100).toFixed(1) + 'pt', c: 'warn', s: `${pct(E.stochastic_capture)} sampled` }
        : live
        ? { k: 'epoch', v: String(Math.round(A.epoch)), c: 'acc', s: `/ ${A.max_epochs || '?'} max` }
        : { k: 'peak→final', v: gapPt != null ? ('−' + gapPt + 'pt') : '—', c: gapPt >= 30 ? 'bad' : gapPt >= 15 ? 'warn' : 'good', s: pct(peak) + ' peak' },
    ];
    cards.innerHTML = rows.map(c =>
      `<div class="stat ${c.c}"><div class="v">${c.v}</div><div class="k">${c.k}</div><div class="s">${c.s}</div></div>`
    ).join('');
  }
  if (capEl) {
    capEl.textContent = ref5inP2
      ? `P1c PASS → held-out P2 STRICT FAIL; timeout ${pct(E.deterministic_timeout)} > 5%. P3 not started.`
      : riskcapFinal
      ? `Trained ep ${E.epoch} + riskcap · unseen seed45 n=${E.heldout_episodes}; seed46 fixed-speed 3/3 PASS.`
      : finalAudit
      ? `Frozen ep ${E.epoch} · deterministic held-out n=${E.deterministic_episodes}; stochastic n=${E.stochastic_episodes}.`
      : live
      ? `Live = last ${A.tail_epochs || 50} epochs · current curriculum state.`
      : `peak ${pct(peak)} (ep ${L.peak_captured_epoch}) → final ${pct(L.last_captured_rate)}.`;
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
    const eyebrow = experiment.generalization_pass === true ? 'FROZEN RESULT' : 'CURRENT HYPOTHESIS';
    const active = experiment.is_live
      ? '<span class="update-live">RUNNING</span>'
      : (experiment.generalization_pass === true
        ? '<span class="update-snap">FINAL PASS</span>'
        : (experiment.ab_experiment
        ? `<span class="update-snap">${experiment.ab_gate_complete ? (experiment.ab_gate_pass ? 'A/B PASS' : 'A/B CLOSED') : 'A/B GATE'}</span>`
        : (experiment.causal_checks_complete
        ? '<span class="update-snap">CAUSAL COMPLETE</span>'
        : (experiment.causal_checks_1to3_complete
        ? '<span class="update-snap">CAUSAL 1–3</span>'
        : (experiment.core_audit_complete
        ? '<span class="update-snap">CORE AUDIT</span>'
        : '<span class="update-snap">SNAPSHOT</span>')))));
    hero.innerHTML = `<div><span class="eyebrow">${eyebrow}</span>
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
      const hasUnique = rows.some(r => r.unique != null);
      evalTbl.innerHTML = `<thead><tr><th>condition</th><th>bars</th><th>capture</th>${hasUnique ? '<th>unique</th>' : ''}<th>verdict</th></tr></thead>
        <tbody>${rows.map(r => `<tr><td>${r.label || '—'}</td>
          <td>${r.bars ?? '—'}</td><td>${r.capture == null ? '—' : pct(r.capture)}</td>
          ${hasUnique ? `<td>${r.unique == null ? '—' : Number(r.unique).toFixed(1)}</td>` : ''}
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

function renderSim2Real72h(s) {
  const p = s.sim2real_72h || {};
  const evidence = p.evidence || {};
  const sim = p.simulation_verification || {};
  const hero = document.getElementById('sim2real-hero');
  const days = document.getElementById('sim2real-days');
  const blockers = document.getElementById('sim2real-blockers');
  if (hero) {
    const verdict = evidence.verdict || 'sensor contract pending';
    const delta = Number(evidence.never_acquired_delta_pp);
    hero.innerHTML = `<div><span class="eyebrow">${p.status || 'MEASURE BEFORE RETRAINING'}</span>
      <strong>실기 숫자를 먼저 닫고, 그 분포로 다음 학습을 사전등록합니다.</strong>
      <p>시뮬레이션 검증: <b>${sim.status || '—'}</b> · software preflight ${sim.preflight_claim_status || '—'}.
      Stage 1: <b>${verdict}</b> · primary ${Number.isFinite(delta) ? delta.toFixed(3) + ' pp' : '—'}
      · Stage 2 ${evidence.stage2_authorised === true ? 'authorised' : 'blocked'}.
      28 m arm의 exact analytic range는 실기 증거가 아닙니다.</p></div>
      <div class="update-run"><span class="update-snap">72 HOUR GATE</span>
      <b>${p.as_of || '—'}</b><span>실기 비행 0회 · physical gate ${sim.physical_gate || '—'} · 새 PPO 보류</span></div>`;
  }
  if (days) {
    days.innerHTML = (p.days || []).map(d => `<article class="milestone">
      <span>${d.day || ''}</span><b>${d.title || ''}</b><p>${d.detail || ''}</p>
      </article>`).join('');
  }
  if (blockers) {
    const software = p.software_readiness || {};
    const softwareHtml = software.status
      ? `<p><b>소프트웨어 준비</b> ${software.status} · ${software.tests || '—'} · ${software.claim_status || '—'}</p>
         <p class="hint">telemetry: ${software.tool || '—'} · ${software.next || ''}</p>`
      : '';
    const simHtml = sim.status
      ? `<p><b>시뮬레이션 상태</b> ${sim.status} · physical gate ${sim.physical_gate || '—'} · mode probe ${sim.mode_probe_verdict || '—'}</p>`
      : '';
    blockers.innerHTML = softwareHtml + simHtml + `<p><b>학습 전 필수</b></p><ul>${(p.training_blockers || [])
      .map(item => `<li>${item}</li>`).join('')}</ul>
      <p class="decision">하나라도 비면 Stage 2/P3/fresh PPO를 실행하지 않습니다.
      <a href="../SIM2REAL_3DAY_EXECUTION_PLAN.md">전체 계측 계약 보기</a></p>`;
  }
}

function renderRuns(s) {
  // Sort by completion time, not by name. update_status_snapshot.py sorts `runs` by run NAME, and
  // this used to just .reverse() that -- so "the latest 12" was the lexicographically last 12, and
  // legacy names like density_120 / density_25 sorted before every ppo_* run.
  const runs = (s.runs || []).slice()
    .sort((a, b) => String(b.finalized_at || '').localeCompare(String(a.finalized_at || '')))
    .slice(0, 12);
  const tbl = document.getElementById('runs-tbl');
  if (!tbl) return;
  tbl.innerHTML = `<thead><tr><th>run</th><th>date</th><th>bars</th><th>epochs</th><th>peak</th><th>final</th><th>crash</th><th>Δ</th><th>exit</th></tr></thead>
    <tbody>${runs.map(r => {
      const cap = r.last_captured_rate, peak = r.peak_captured_rate;
      const gap = (peak != null && cap != null) ? (peak - cap) : null;
      const hi = r.run === (s.latest_run && s.latest_run.run) ? ' class="hi"' : '';
      // reward_collapse was carried in the snapshot but never surfaced -- a collapsed run's
      // numbers should not be read the same way as a clean one's.
      const exit = r.reward_collapse ? '<b class="bad-ink">collapse</b>' : (r.exit_reason || '—');
      return `<tr${hi}><td>${(r.run || '').replace(/_navrl$/, '')}</td>
        <td>${(r.finalized_at || '').slice(0, 10)}</td>
        <td>${r.last_n_bars_active ?? '—'}</td>
        <td>${r.epochs_logged ?? '—'}</td>
        <td>${pct(peak)}</td><td>${pct(cap)}</td><td>${pct(r.last_crash_rate)}</td>
        <td>${gap != null ? ('−' + (gap * 100).toFixed(0) + 'pt') : '—'}</td>
        <td>${exit}</td></tr>`;
    }).join('')}</tbody>`;
}

function renderNow(s) {
  const u = s.research_update || {};
  const experiment = u.active_experiment || u.bounded_pilot || {};
  const set = (id, value) => {
    const el = document.getElementById(id);
    if (el) el.textContent = value || '—';
  };
  set('now-latest', u.headline);
  set('now-status', u.summary);
  set('now-next', u.decision);

  const verification = document.getElementById('now-verification');
  if (verification) {
    const verifiedAt = experiment.recovery_attestation_verified_at;
    verification.hidden = !verifiedAt;
    verification.textContent = verifiedAt
      ? `Evidence verified for static snapshot ${verifiedAt}. The safe launcher rechecks the live files before training.`
      : '';
  }
}

function renderPhases(s) {
  const u = s.research_update || {};
  const experiment = u.active_experiment || u.bounded_pilot || {};
  const next = (u.milestones || []).find(m => String(m.label || '').toUpperCase() === 'NEXT');
  const reachedBudget = Number.isFinite(Number(experiment.epoch))
    && Number.isFinite(Number(experiment.max_epochs))
    && Number(experiment.epoch) >= Number(experiment.max_epochs);
  const currentDone = experiment.core_audit_complete === true
    || experiment.generalization_pass === true
    || experiment.recovery_attestation_valid === true
    || (Boolean(experiment.ab_arm) && !experiment.is_live && reachedBudget);
  const currentLabel = String(u.subtitle || 'current research stage')
    .replace(/^\d{4}-\d{2}-\d{2}\s*·\s*/, '');
  const nextLabel = next
    ? [next.value, next.detail].filter(Boolean).join(' · ')
    : (u.decision || 'next gated experiment');
  const phases = [
    ['Done', 'validated foundation and prior ablations', 'done'],
    ['Now', currentLabel, currentDone ? 'done' : 'active'],
    ['Next', nextLabel, currentDone ? 'active' : 'todo'],
    ['Paper', 'held-out ablation + write-up', 'todo'],
  ];
  const ph = document.getElementById('phases');
  if (!ph) return;
  const lab = { done: 'done', active: 'now', todo: 'todo' };
  ph.replaceChildren(...phases.map(p => {
    const row = document.createElement('div');
    row.className = `phase${p[2] === 'active' ? ' is-active' : ''}`;
    const id = document.createElement('span');
    id.className = 'pid';
    id.textContent = p[0];
    const title = document.createElement('span');
    title.className = 'pt';
    title.textContent = p[1];
    const badge = document.createElement('span');
    badge.className = `badge ${p[2]}`;
    badge.textContent = lab[p[2]];
    row.append(id, title, badge);
    return row;
  }));
}

MOTAR.register(renderLive, renderCriteria, renderResearchUpdate, renderSim2Real72h, renderRuns, renderNow,
               renderPhases);
