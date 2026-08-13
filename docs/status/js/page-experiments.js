/* experiments.html — the controlled-experiment index.
 *
 * One row per experiment: what was changed, against what, the effect with its CI, and two separate
 * chips -- the verdict on the HYPOTHESIS and the validity of the EVIDENCE. Those are orthogonal
 * (a result can have passed its gate and still be void), and collapsing them would force
 * retractions to be labelled failures, which misreports the record.
 *
 * Default filter is validity=canonical. The rest is one click away and labelled with a count,
 * because for a reviewer audience the retracted set is evidence that the verification worked, not
 * something to bury.
 */
(function () {
  'use strict';

  var DATA = window.__EXPERIMENTS__;
  var root = document.getElementById('exp-root');
  if (!root || !DATA) return;

  var esc = function (s) {
    return String(s == null ? '' : s).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  };

  var EXPS = DATA.experiments.slice().sort(function (a, b) {
    return String(b.worklog_date).localeCompare(String(a.worklog_date));
  });
  var BY_ID = {};
  EXPS.forEach(function (e) { BY_ID[e.id] = e; });
  var THEME = {};
  DATA.themes.forEach(function (t) { THEME[t.id] = t.label_ko; });

  var VALIDITY_LABEL = {
    canonical: '유효', superseded: '대체됨', withdrawn: '철회', exploratory: '탐색적'
  };
  var RETRACTION_LABEL = {
    bug: '구현 버그', confound: '교란', rng_contamination: 'RNG 오염',
    wrong_frame: '좌표계 오류', spec_error: '과대주장', scope: '범위 이탈'
  };

  // Shareable filter state: experiments.html?validity=void is the link to hand someone who asks
  // "what did you throw away", and ?theme=latency scopes a discussion to one axis.
  var qs = new URLSearchParams(location.search);
  var state = {
    theme: qs.get('theme') || 'all',
    validity: qs.get('validity') || 'canonical',
    q: ''
  };

  function effectText(e) {
    var f = e.effect || {};
    if (f.value_pp === null || f.value_pp === undefined) {
      return '<span class="dim">' + esc(e.metric_label || '—') + '</span>';
    }
    var sign = f.value_pp > 0 ? '+' : '';
    var good = (f.direction === 'lower_is_better') ? f.value_pp < 0 : f.value_pp > 0;
    var cls = Math.abs(f.value_pp) < 1e-9 ? 'dim' : (good ? 'good-ink' : 'bad-ink');
    var h = '<b class="' + cls + '">' + sign + f.value_pp.toFixed(2) + ' pp</b>';
    if (f.ci95_pp) {
      var crosses = f.ci95_pp[0] <= 0 && f.ci95_pp[1] >= 0;
      h += ' <span class="dim">[' + f.ci95_pp[0].toFixed(2) + ', ' + f.ci95_pp[1].toFixed(2) + ']'
         + (crosses ? ' · 0 포함' : '') + '</span>';
    }
    return h;
  }

  function verdictChip(v) {
    var cls = v === 'PASS' ? 'chip-good' : v === 'FAIL' ? 'chip-bad' : 'chip-warn';
    return '<span class="chip ' + cls + '">' + esc(v) + '</span>';
  }

  function validityChip(v) {
    if (v === 'canonical') return '';
    var cls = v === 'withdrawn' ? 'chip-bad' : 'chip-warn';
    return '<span class="chip ' + cls + ' chip-struck">' + esc(VALIDITY_LABEL[v] || v) + '</span>';
  }

  function armsTable(e) {
    if (!e.arms || !e.arms.length) return '';
    var extraKeys = {};
    e.arms.forEach(function (a) { Object.keys(a.extra || {}).forEach(function (k) { extraKeys[k] = 1; }); });
    var cols = ['capture', 'crash', 'timeout', 'n_episodes'].filter(function (c) {
      return e.arms.some(function (a) { return a[c] !== undefined && a[c] !== null; });
    });
    var ex = Object.keys(extraKeys);
    var pctv = function (v) { return (v == null) ? '—' : (100 * v).toFixed(2) + '%'; };
    var head = ['arm'].concat(cols.map(function (c) {
      return { capture: 'capture', crash: 'crash', timeout: 'timeout', n_episodes: 'n' }[c];
    })).concat(ex);
    return '<h4>측정된 셀</h4><div class="scrollx"><table><thead><tr>'
      + head.map(function (h) { return '<th>' + esc(h) + '</th>'; }).join('')
      + '</tr></thead><tbody>'
      + e.arms.map(function (a) {
          return '<tr><td>' + esc(a.label) + '</td>'
            + cols.map(function (c) {
                var v = a[c];
                return '<td>' + (c === 'n_episodes' ? (v == null ? '—' : v) : pctv(v)) + '</td>';
              }).join('')
            + ex.map(function (k) {
                return '<td>' + esc((a.extra || {})[k] === undefined ? '—' : (a.extra || {})[k]) + '</td>';
              }).join('')
            + '</tr>';
        }).join('')
      + '</tbody></table></div>';
  }

  function link(id) {
    var o = BY_ID[id];
    return '<a href="#' + esc(id) + '">' + esc(o ? o.title_ko : id) + '</a>';
  }

  function detail(e) {
    var h = '<div class="pdetail">';
    h += '<p class="exp-q">' + esc(e.question) + '</p>';
    h += '<div class="pdoc">' + esc(e.verdict_note) + '</div>';

    if (e.retraction) {
      h += '<div class="exp-retract"><b>철회 · ' + esc(RETRACTION_LABEL[e.retraction.reason] || e.retraction.reason)
         + '</b> <span class="dim">' + esc(e.retraction.found_at || '')
         + (e.retraction.found_by ? ' · ' + esc(e.retraction.found_by === 'codex' ? '검수 세션 지적' : '자체 발견') : '')
         + '</span><br>' + esc(e.retraction.text) + '</div>';
    }
    if (e.confounded_by) {
      h += '<div class="exp-confound"><b>교란</b><br>' + esc(e.confounded_by) + '</div>';
    }

    h += '<div class="pmeta"><div>';
    h += '<h4>비교한 수준</h4><ul>';
    (e.levels || []).forEach(function (l) {
      h += '<li>' + esc(l.label) + (l.is_baseline ? ' <span class="chip">기준</span>' : '') + '</li>';
    });
    h += '</ul>';

    h += '<h4>측정 조건</h4><ul>';
    if (e.seeds && e.seeds.length) h += '<li>시드 <code>' + e.seeds.join(', ') + '</code></li>';
    if (e.episodes_per_cell) h += '<li>셀당 <code>' + e.episodes_per_cell.toLocaleString() + '</code> 에피소드</li>';
    if (e.env) {
      if (e.env.bars !== undefined && e.env.bars !== null) h += '<li>막대 <code>' + e.env.bars + '</code>개</li>';
      if (e.env.target_speed_ms != null) h += '<li>표적 속도 <code>' + e.env.target_speed_ms + ' m/s</code></li>';
      if (e.env.arena) h += '<li>아레나 ' + esc(e.env.arena) + '</li>';
      if (e.env.machine) {
        h += '<li>평가 장비 <code>' + esc(e.env.machine) + '</code>'
           + (e.env.machine === 'gtx1650ti'
              ? ' <span class="chip chip-bad">본 장비와 혼합 금지</span>' : '') + '</li>';
      }
    }
    if (e.policy && e.policy.checkpoint) {
      h += '<li>정책 <code>' + esc(e.policy.checkpoint) + '</code>'
         + (e.policy.sha256_prefix ? ' <span class="dim">' + esc(e.policy.sha256_prefix) + '…</span>' : '')
         + '</li>';
    }
    h += '<li>사전등록 ' + (e.preregistered ? '<b class="good-ink">예</b>' : '<b class="warn-ink">아니오</b>') + '</li>';
    if (e.gate) h += '<li>게이트 <code>' + esc(e.gate) + '</code></li>';
    h += '</ul></div><div>';

    h += armsTable(e);

    var rel = [];
    if (e.superseded_by) rel.push('<li>대체됨 → ' + link(e.superseded_by) + '</li>');
    (e.supersedes || []).forEach(function (s) { rel.push('<li>대체함 ← ' + link(s) + '</li>'); });
    if (rel.length) h += '<h4>계보</h4><ul>' + rel.join('') + '</ul>';

    var prov = [];
    if (e.knob) prov.push('<li>knob <a href="parameters.html#' + esc(e.knob) + '"><code>' + esc(e.knob) + '</code></a></li>');
    (e.knob_extra || []).forEach(function (k) {
      prov.push('<li>함께 고정 <a href="parameters.html#' + esc(k) + '"><code>' + esc(k) + '</code></a></li>');
    });
    if (e.launcher) prov.push('<li>런처 <code>' + esc(e.launcher) + '</code></li>');
    (e.results_paths || []).forEach(function (p) { prov.push('<li>원자료 <code>' + esc(p) + '</code></li>'); });
    prov.push('<li>WORKLOG <code>' + esc(e.worklog_date) + '</code></li>');
    h += '<h4>출처</h4><ul>' + prov.join('') + '</ul>';

    if (e._note) h += '<p class="hint">' + esc(e._note) + '</p>';
    h += '</div></div></div>';
    return h;
  }

  function matches(e) {
    if (state.theme !== 'all' && e.theme !== state.theme) return false;
    if (state.validity === 'canonical' && e.validity !== 'canonical') return false;
    if (state.validity === 'void' && (e.validity === 'canonical' || e.validity === 'exploratory')) return false;
    if (state.q) {
      var hay = (e.title_ko + ' ' + e.question + ' ' + e.verdict_note + ' ' + (e.knob || '')).toLowerCase();
      if (hay.indexOf(state.q) === -1) return false;
    }
    return true;
  }

  function render() {
    var rows = EXPS.filter(matches);
    var h = '<p class="hint">' + rows.length + ' / ' + EXPS.length + '건 표시</p><div class="ptable">';
    rows.forEach(function (e) {
      h += '<details class="prow exp-row" id="' + esc(e.id) + '">'
         + '<summary>'
         + '<span class="exp-date">' + esc(e.worklog_date) + '</span>'
         + '<span class="exp-title' + (e.validity === 'withdrawn' ? ' struck' : '') + '">'
         + esc(e.title_ko) + '</span>'
         + '<span class="exp-knob">' + (e.knob ? '<code>' + esc(e.knob) + '</code>'
                                               : '<span class="dim">코드 변경</span>') + '</span>'
         + '<span class="exp-effect">' + effectText(e) + '</span>'
         + '<span class="exp-chips">' + verdictChip(e.verdict) + ' ' + validityChip(e.validity)
         + ' <span class="chip">' + esc(THEME[e.theme] || e.theme) + '</span></span>'
         + '</summary>' + detail(e) + '</details>';
    });
    h += '</div>';
    root.innerHTML = h;
    openHash();
  }

  function openHash() {
    var id = decodeURIComponent(location.hash.slice(1));
    if (!id) return;
    var el = document.getElementById(id);
    if (el) { el.open = true; el.scrollIntoView({ block: 'center' }); }
  }

  function wire() {
    var c = document.getElementById('exp-controls');
    var nVoid = EXPS.filter(function (e) {
      return e.validity === 'superseded' || e.validity === 'withdrawn';
    }).length;

    var on = function (key, val) { return state[key] === val ? ' on' : ''; };
    var themes = ['<button class="seg-btn' + on('theme', 'all') + '" data-theme="all">전체</button>'];
    DATA.themes.forEach(function (t) {
      if (!EXPS.some(function (e) { return e.theme === t.id; })) return;
      themes.push('<button class="seg-btn' + on('theme', t.id) + '" data-theme="' + t.id + '">'
                  + esc(t.label_ko) + '</button>');
    });

    c.innerHTML = '<div class="seg">' + themes.join('') + '</div>'
      + '<div class="seg">'
      + '<button class="seg-btn' + on('validity', 'canonical') + '" data-validity="canonical">유효한 근거</button>'
      + '<button class="seg-btn' + on('validity', 'void') + '" data-validity="void">검증으로 무효화 ' + nVoid + '</button>'
      + '<button class="seg-btn' + on('validity', 'all') + '" data-validity="all">전체</button>'
      + '</div>'
      + '<input class="pfind" id="exp-find" type="search" placeholder="제목 · 질문 · knob 검색" aria-label="실험 검색">';

    c.addEventListener('click', function (ev) {
      var b = ev.target.closest('.seg-btn');
      if (!b) return;
      var key = b.dataset.theme ? 'theme' : 'validity';
      state[key] = b.dataset[key];
      b.parentNode.querySelectorAll('.seg-btn').forEach(function (x) { x.classList.toggle('on', x === b); });
      render();
    });
    document.getElementById('exp-find').addEventListener('input', function (ev) {
      state.q = ev.target.value.trim().toLowerCase();
      render();
    });
  }

  function summary() {
    var el = document.getElementById('exp-summary');
    if (!el) return;
    var v = {}, d = {};
    EXPS.forEach(function (e) {
      v[e.validity] = (v[e.validity] || 0) + 1;
      d[e.verdict] = (d[e.verdict] || 0) + 1;
    });
    el.innerHTML = '통제 실험 <b>' + EXPS.length + '</b>건 · '
      + 'PASS ' + (d.PASS || 0) + ' · FAIL ' + (d.FAIL || 0) + ' · 미결 ' + (d.INCONCLUSIVE || 0)
      + ' <span class="dim">|</span> '
      + '유효 ' + (v.canonical || 0) + ' · 대체됨 ' + (v.superseded || 0)
      + ' · 철회 ' + (v.withdrawn || 0) + ' · 탐색적 ' + (v.exploratory || 0);
  }

  wire();
  summary();
  render();
  window.addEventListener('hashchange', openHash);
})();
