/* parameters.html — the NAVRL_* catalogue, rendered from data/parameters.json.
 *
 * Master/detail: one scannable row per knob, expanding in place to the source comment, every
 * declaration site, and the launchers that set it. The point of the page is to answer "what does
 * this knob do, what is it set to, and did we ever actually test it" without opening the repo.
 */
(function () {
  'use strict';

  var DATA = window.__PARAMETERS__;
  var root = document.getElementById('param-root');
  if (!root || !DATA) return;

  var EXP = (window.__EXPERIMENTS__ || {}).experiments || [];
  var EXP_BY_ID = {};
  EXP.forEach(function (e) { EXP_BY_ID[e.id] = e; });

  var esc = function (s) {
    return String(s == null ? '' : s).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  };

  var state = { group: 'all', flag: 'all', q: '' };

  function fmtDefault(p) {
    if (p.default_expr) return '<code>' + esc(p.default_expr) + '</code>';
    var v = p.effective_default;
    if (v === null || v === undefined) return '<span class="dim">—</span>';
    if (v === '') return '<code class="dim">""</code>';
    return '<code>' + esc(v) + '</code>' + (p.unit ? ' <span class="dim">' + esc(p.unit) + '</span>' : '');
  }

  function statusChips(p) {
    var out = [];
    if (p.ablated) {
      out.push('<span class="chip chip-good">실험 ' + p.experiments.length + '건</span>');
    } else if (p.swept_but_not_ablated) {
      // The honest third state: the launchers set it, but no controlled A/B ever isolated it.
      out.push('<span class="chip chip-warn">설정만 · 통제 A/B 없음</span>');
    } else {
      out.push('<span class="chip dim">기본값 고정</span>');
    }
    if (p.mirrors) out.push('<span class="chip chip-bad">' + p.declarations.length + '곳 선언 · 동기화 필요</span>');
    if (p.frozen_by_contract) out.push('<span class="chip chip-lock">관측 계약 고정</span>');
    if (p.effective_differs) out.push('<span class="chip dim">후처리 기본값</span>');
    return out.join(' ');
  }

  function detail(p) {
    var h = '<div class="pdetail">';
    if (p.doc) {
      h += '<div class="pdoc">' + esc(p.doc).replace(/\n/g, '<br>') + '</div>';
    } else {
      h += '<p class="hint">소스에 주석이 없다.</p>';
    }
    h += '<div class="pmeta">';
    h += '<div><h4>선언 위치' + (p.mirrors ? ' <span class="chip chip-bad">' + p.declarations.length + '곳</span>' : '') + '</h4><ul>';
    p.declarations.forEach(function (d) {
      h += '<li><code>' + esc(d.file) + ':' + d.line + '</code>'
         + (d.scope && d.scope !== '<module>' ? ' <span class="dim">' + esc(d.scope) + '</span>' : '')
         + ' <span class="dim">= ' + esc(String(d.default)) + '</span></li>';
    });
    h += '</ul>';
    if (p.attr) h += '<p class="hint">파이썬 경로 <code>' + esc(p.attr) + '</code></p>';
    if (p.echoes && p.echoes.length) {
      h += '<p class="hint">' + p.echoes.length + '곳에서 영수증·매니페스트용으로 다시 읽는다 (동작에 영향 없음).</p>';
    }
    h += '</div>';

    h += '<div><h4>실험</h4>';
    if (p.experiments.length) {
      h += '<ul>';
      p.experiments.forEach(function (id) {
        var e = EXP_BY_ID[id];
        h += '<li><a href="experiments.html#' + esc(id) + '">'
           + esc(e ? e.title_ko : id) + '</a>'
           + (e ? ' <span class="chip chip-' + (e.verdict === 'PASS' ? 'good' : e.verdict === 'FAIL' ? 'bad' : 'warn')
                  + '">' + esc(e.verdict) + '</span>' : '') + '</li>';
      });
      h += '</ul>';
    } else if (p.swept_but_not_ablated) {
      h += '<p class="hint">통제 A/B로 분리한 적이 없다. 아래 런처들이 값을 지정하지만, 그것만으로는 '
         + '이 knob의 효과를 다른 변화와 분리할 수 없다.</p>';
    } else {
      h += '<p class="hint">한 번도 바꿔본 적이 없다.</p>';
    }
    if (p.launchers && p.launchers.length) {
      var uniq = {};
      p.launchers.forEach(function (l) { uniq[l.value] = (uniq[l.value] || 0) + 1; });
      h += '<h4>런처에서 지정한 값</h4><ul class="pvals">';
      Object.keys(uniq).sort().forEach(function (v) {
        h += '<li><code>' + esc(v) + '</code> <span class="dim">× ' + uniq[v] + '</span></li>';
      });
      h += '</ul>';
    }
    h += '</div></div></div>';
    return h;
  }

  function matches(p) {
    if (state.group !== 'all' && p.group !== state.group) return false;
    if (state.flag === 'ablated' && !p.ablated) return false;
    if (state.flag === 'swept' && !p.swept_but_not_ablated) return false;
    if (state.flag === 'mirrors' && !p.mirrors) return false;
    if (state.flag === 'frozen' && !p.frozen_by_contract) return false;
    if (state.q) {
      var hay = (p.name + ' ' + (p.doc || '') + ' ' + (p.attr || '')).toLowerCase();
      if (hay.indexOf(state.q) === -1) return false;
    }
    return true;
  }

  var LABEL = {};
  DATA.groups.forEach(function (g) { LABEL[g.id] = g.label_ko; });

  function render() {
    var rows = DATA.parameters.filter(matches);
    var byGroup = {};
    rows.forEach(function (p) { (byGroup[p.group] = byGroup[p.group] || []).push(p); });

    var h = '<p class="hint" id="param-count">' + rows.length + ' / ' + DATA.parameters.length + '개 표시</p>';
    DATA.groups.forEach(function (g) {
      var list = byGroup[g.id];
      if (!list || !list.length) return;
      h += '<h3 class="pgroup">' + esc(g.label_ko) + ' <span class="dim">' + list.length + '</span></h3>';
      h += '<div class="ptable">';
      list.forEach(function (p) {
        h += '<details class="prow" id="' + esc(p.name) + '">'
           + '<summary>'
           + '<code class="pname">' + esc(p.name) + '</code>'
           + '<span class="pval">' + fmtDefault(p) + '</span>'
           + '<span class="pchips">' + statusChips(p) + '</span>'
           + '<span class="pdesc">' + esc((p.doc_short || '').slice(0, 120)) + '</span>'
           + '</summary>' + detail(p) + '</details>';
      });
      h += '</div>';
    });
    root.innerHTML = h;

    // Deep link: parameters.html#NAVRL_X opens that knob.
    var hash = decodeURIComponent(location.hash.slice(1));
    if (hash) {
      var el = document.getElementById(hash);
      if (el) { el.open = true; el.scrollIntoView({ block: 'center' }); }
    }
  }

  function wire() {
    var c = document.getElementById('param-controls');
    var groups = ['<button class="seg-btn on" data-group="all">전체</button>'];
    DATA.groups.forEach(function (g) {
      if (!DATA.parameters.some(function (p) { return p.group === g.id; })) return;
      groups.push('<button class="seg-btn" data-group="' + g.id + '">' + esc(g.label_ko) + '</button>');
    });
    var flags = [
      ['all', '전체'],
      ['ablated', '실험함'],
      ['swept', '설정만'],
      ['mirrors', '동기화 위험'],
      ['frozen', '계약 고정']
    ].map(function (f, i) {
      return '<button class="seg-btn' + (i === 0 ? ' on' : '') + '" data-flag="' + f[0] + '">' + f[1] + '</button>';
    });

    c.innerHTML = '<div class="seg">' + groups.join('') + '</div>'
                + '<div class="seg">' + flags.join('') + '</div>'
                + '<input class="pfind" id="param-find" type="search" placeholder="이름 · 주석 검색" aria-label="파라미터 검색">';

    c.addEventListener('click', function (e) {
      var b = e.target.closest('.seg-btn');
      if (!b) return;
      var key = b.dataset.group ? 'group' : 'flag';
      state[key] = b.dataset[key];
      b.parentNode.querySelectorAll('.seg-btn').forEach(function (x) { x.classList.toggle('on', x === b); });
      render();
    });
    document.getElementById('param-find').addEventListener('input', function (e) {
      state.q = e.target.value.trim().toLowerCase();
      render();
    });
  }

  function stamp() {
    var el = document.getElementById('param-stamp');
    if (!el) return;
    var c = DATA.counts;
    el.innerHTML = '소스에서 생성 · <code>' + esc(DATA.generator) + '</code> · '
      + c.files_scanned + '개 파일, 런처 ' + c.launchers_scanned + '개 스캔 · '
      + '동작 knob <b>' + c.authoritative + '</b>개'
      + (DATA.source_commit ? ' · <code>' + esc(DATA.source_commit) + '</code>' : '');
  }

  function renderEchoes() {
    var el = document.getElementById('echo-root');
    var n = document.getElementById('echo-count');
    if (n) n.textContent = (DATA.echo_only || []).length;
    if (!el) return;
    el.innerHTML = '<ul class="pvals">' + (DATA.echo_only || []).map(function (e) {
      return '<li><code>' + esc(e.name) + '</code> <span class="dim">'
           + e.sites.length + '곳 · ' + esc(e.sites[0].file.split('/').pop()) + ':' + e.sites[0].line
           + '</span></li>';
    }).join('') + '</ul>';
  }

  wire();
  stamp();
  render();
  renderEchoes();
  window.addEventListener('hashchange', render);
})();
