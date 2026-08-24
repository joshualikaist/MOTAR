/* MOTAR dashboard shell — sidebar, top bar, footer, theme, live pill.
 *
 * Injected at runtime rather than duplicated into six HTML files. The no-JS argument for static
 * markup is already spent here: every panel on every page is an empty <div> filled by a renderer,
 * so the pages are blank skeletons without JS regardless. What duplication WOULD buy us is six
 * copies of a nav that nothing guards -- there is no build step and no CI that opens the HTML, so
 * the first page rename would silently desync them. This repo has been bitten by exactly that
 * failure class before (navrl_task_config.py:210-213 documents duplicated constants that "were
 * read by nobody -- editing them looked like tuning but changed nothing").
 *
 * Load position: in <head>, right after style.css. It has to run before first paint so the stored
 * theme is applied without a flash; the chrome itself is built on DOMContentLoaded, once <body>
 * exists. Data and renderers stay at the end of <body>. All classic scripts -- no defer, no
 * type=module (both imply deferral and would silently reorder against status.fallback.js).
 */
(function () {
  'use strict';

  var NAV = [
    { href: 'index.html',       label: '개요',        desc: '한 줄 결론과 라이브 상태' },
    { href: 'system.html',      label: '시스템 구조', desc: 'high/low-level과 자료구조' },
    { href: 'setup.html',       label: '초기 세팅',   desc: '과제 · 관측 계약 · 아레나' },
    { href: 'drone.html',       label: '기체 · 센싱', desc: '플랫폼과 센서, 실기 격차' },
    { href: 'parameters.html',  label: '파라미터',    desc: 'NAVRL_* 카탈로그' },
    { href: 'experiments.html', label: '실험 인덱스', desc: '통제 실험 전량' },
    { href: 'results.html',     label: '결과',        desc: '정식 근거만' }
  ];

  // ---- theme -----------------------------------------------------------------------------
  // Persisted, unlike the pre-split version: with one page a reset on reload was invisible, but
  // across six pages an un-persisted theme resets on every navigation.
  var THEME_KEY = 'motar.theme';

  function applyTheme(name) {
    document.documentElement.setAttribute('data-theme', name);
    try { localStorage.setItem(THEME_KEY, name); } catch (e) { /* private mode */ }
    if (window.__mo && window.__mo.recolor) window.__mo.recolor();
  }

  function initTheme() {
    var saved = null;
    try { saved = localStorage.getItem(THEME_KEY); } catch (e) { /* private mode */ }
    if (!saved) {
      saved = matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
    }
    document.documentElement.setAttribute('data-theme', saved);
  }

  function toggleTheme() {
    var cur = document.documentElement.getAttribute('data-theme') || 'light';
    applyTheme(cur === 'dark' ? 'light' : 'dark');
  }

  // Exported for any page that wants to bind its own control; shell.js binds #themebtn itself.
  window.toggleTheme = toggleTheme;

  // ---- markup ----------------------------------------------------------------------------
  function currentPage() {
    var file = (location.pathname.split('/').pop() || 'index.html');
    return file === '' ? 'index.html' : file;
  }

  function navHtml(here) {
    return NAV.map(function (item) {
      var on = item.href === here ? ' class="on"' : '';
      var cur = item.href === here ? ' aria-current="page"' : '';
      return '<a href="' + item.href + '"' + on + cur + '>' +
             '<span class="side-l">' + item.label + '</span>' +
             '<span class="side-d">' + item.desc + '</span></a>';
    }).join('');
  }

  function build() {
    var here = currentPage();
    var title = document.body.dataset.pageTitle || '';

    document.body.insertAdjacentHTML('afterbegin',
      '<a class="skip-link" href="#main">본문으로 이동</a>' +
      '<button class="side-toggle" id="side-toggle" type="button" aria-label="메뉴" ' +
        'aria-expanded="false" aria-controls="side">☰</button>' +
      '<aside class="side" id="side">' +
        '<a class="brand" href="index.html">MOTAR</a>' +
        '<p class="side-sub">센서 전용 UAV 요격 · <code>research/navrl-env</code></p>' +
        '<nav class="side-nav" aria-label="페이지">' + navHtml(here) + '</nav>' +
        '<div class="side-foot">' +
          '<button class="themebtn" id="themebtn" type="button" aria-label="테마 전환">◐</button>' +
          '<span class="side-gen">updated <span id="foot-gen">—</span></span>' +
        '</div>' +
      '</aside>' +
      '<div class="side-scrim" id="side-scrim" hidden></div>' +
      '<header class="top">' +
        '<h1 class="top-title">' + title + '</h1>' +
        '<span class="pill is-live" id="livepill">' +
          '<span class="dot"></span><span id="livepill-txt">LIVE</span>' +
          '<b id="livepill-run">—</b>' +
        '</span>' +
        '<span class="top-context" id="top-context" aria-live="polite">v2 · loading current state</span>' +
      '</header>'
    );

    document.getElementById('themebtn').addEventListener('click', toggleTheme);

    var toggle = document.getElementById('side-toggle');
    var scrim = document.getElementById('side-scrim');
    function setDrawer(open) {
      document.body.classList.toggle('side-open', open);
      toggle.setAttribute('aria-expanded', String(open));
      scrim.hidden = !open;
    }
    toggle.addEventListener('click', function () {
      setDrawer(!document.body.classList.contains('side-open'));
    });
    scrim.addEventListener('click', function () { setDrawer(false); });
    document.addEventListener('keydown', function (e) {
      if (e.key === 'Escape') setDrawer(false);
    });

    // Paint the pill from the snapshot immediately. The page renderers run in an IIFE at the end
    // of <body>, i.e. BEFORE this DOMContentLoaded handler, so their first pass writes into a
    // header that does not exist yet. Their second pass (after fetch) would fix it -- but only if
    // the fetch succeeds, which it never does over file://.
    renderPill(window.__STATUS_FALLBACK__);
  }

  // ---- live pill -------------------------------------------------------------------------
  // MOVED here from renderLive (not copied). The pill, the context line and the generated-at stamp
  // describe the snapshot, not the Live panel, and they must be identical on all six pages -- a
  // per-page copy of this branching would drift immediately.
  //
  // The `ref5inP2 ? ... : riskcapFinal ? ...` chain exists because `research_update` is polymorphic
  // across five branches of update_status_snapshot.py. It is carried verbatim for now; once
  // data/experiments.json is the source of truth this collapses to "render the experiment whose id
  // is research_update.experiment_id".
  var finite = function (v) { var n = Number(v); return Number.isFinite(n) ? n : null; };
  var pct = function (v) { return v == null ? '—' : (100 * v).toFixed(2) + '%'; };

  function renderPill(s) {
    if (!s) return;
    var A = s.active_run;
    var L = s.latest_run || {};
    var live = !!(A && A.is_live);
    var E = (s.research_update || {}).active_experiment || {};
    var ref5inP2 = !live && E.ref5in_p2 === true;
    var finalAudit = !live && E.core_audit_complete === true;
    var riskcapFinal = !live && E.speed_governor_mode === 'riskcap'
      && E.post_evaluation_complete === true && E.generalization_pass === true;
    var causalAudit = finalAudit && E.causal_checks_1to3_complete === true;
    var completeAudit = finalAudit && E.causal_checks_complete === true;
    var abExperiment = E.ab_experiment === true;
    var src = live ? A : L;
    var runName = String(ref5inP2 ? E.run : ((src && src.run) || '—')).replace(/_navrl$/, '');

    var pill = document.getElementById('livepill');
    var txt = document.getElementById('livepill-txt');
    var run = document.getElementById('livepill-run');
    if (pill) pill.className = 'pill ' + (live ? 'is-live' : 'is-snapshot');
    if (txt) {
      txt.textContent = live ? 'LIVE' : (ref5inP2 ? E.p2_verdict : (riskcapFinal ? 'FINAL'
        : (abExperiment ? 'A/B' : (completeAudit ? 'AUDIT'
        : (causalAudit ? 'CAUSAL' : (finalAudit ? 'CORE' : 'LAST'))))));
    }
    if (run) run.textContent = runName;

    var ctx = document.getElementById('top-context');
    if (ctx) {
      var barsNow = live ? finite(A.n_bars_active)
        : ((finalAudit || abExperiment || riskcapFinal) ? finite(E.bars) : finite(L.last_n_bars_active));
      var tailCapture = live ? finite(A.captured_rate)
        : ((abExperiment || riskcapFinal) ? finite(E.heldout_capture)
          : (finalAudit ? finite(E.deterministic_capture) : finite(L.last_captured_rate)));
      var state = live ? 'v2 · LIVE' : (ref5inP2 ? 'corrected-v2 · REF5IN P2 FAIL'
        : (riskcapFinal ? 'v2 · RISKCAP FINAL' : (abExperiment ? 'v2 · FIXED-205 A/B'
        : (completeAudit ? 'v2 · CAUSAL COMPLETE' : (causalAudit ? 'v2 · CAUSAL AUDIT'
        : (finalAudit ? 'v2 · CORE AUDIT' : 'v2 · SNAPSHOT'))))));
      ctx.textContent = state + ' · ' + (barsNow == null ? '—' : Math.round(barsNow)) + ' bars'
        + (tailCapture == null ? '' : ' · capture ' + pct(tailCapture));
    }

    var gen = document.getElementById('foot-gen');
    if (gen) gen.textContent = String(s.generated_at || '').slice(0, 16).replace('T', ' ') || '—';
    var runs = document.getElementById('foot-runs');
    if (runs) runs.textContent = s.n_runs || (s.runs ? s.runs.length : '—');
  }
  // Page renderers call this on their fetch pass so the pill tracks live data, not just the snapshot.
  window.renderShellPill = renderPill;

  if (document.readyState === 'loading') {
    initTheme();
    document.addEventListener('DOMContentLoaded', build);
  } else {
    initTheme();
    build();
  }
})();
