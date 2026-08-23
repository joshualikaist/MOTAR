/* drone.html — platform and sensing.
 *
 * Every number here is derived: the airframe table from data/platform.json (parsed out of the
 * URDFs and robot configs by tools/generate_platform_spec.py), the sensor and control rows from
 * data/parameters.json. Nothing is retyped, because the 2026-08-13 audit found the legacy URDF had
 * been internally inconsistent for three weeks precisely because nobody recomputed it.
 */
(function () {
  'use strict';

  var PL = window.__PLATFORM__;
  var PARAMS = window.__PARAMETERS__;
  if (!PL) return;

  var esc = function (s) {
    return String(s == null ? '' : s).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  };
  var P = {};
  ((PARAMS || {}).parameters || []).forEach(function (p) { P[p.name] = p; });

  function knob(name, fallback) {
    var p = P[name];
    if (!p) return fallback == null ? '—' : fallback;
    var v = p.effective_default;
    return (v === null || v === undefined || v === '') ? (fallback == null ? '—' : fallback) : v;
  }

  function num(v, digits) {
    return (v === null || v === undefined) ? '—' : Number(v).toFixed(digits == null ? 2 : digits);
  }

  // ---- airframe comparison ----------------------------------------------------------------
  function renderAirframe() {
    var el = document.getElementById('drone-airframe');
    if (!el) return;
    var R = PL.robots;
    var rows = [
      ['총 질량', function (r) { return num(r.mass_kg, 3) + ' kg'; }],
      ['모터 팔 (x=y)', function (r) { return num(r.arm_m, 4) + ' m'; }],
      ['모터간 대각', function (r) { return num(r.derived.motor_diagonal_m, 3) + ' m'; }],
      ['충돌 박스 (W×H)', function (r) { return num(r.collision_box_m[0], 2) + ' × ' + num(r.collision_box_m[2], 2) + ' m'; }],
      ['충돌 박스 대각 (XY)', function (r) { return num(r.derived.box_diagonal_m, 3) + ' m'; }],
      ['관성 ixx=iyy (조립)', function (r) { return r.ixx_assembled.toExponential(3); }],
      ['관성 izz (조립)', function (r) { return r.izz_assembled.toExponential(3); }],
      ['모터당 최대 추력', function (r) { return num(r.max_thrust_n, 2) + ' N'; }],
      ['총 추력', function (r) { return num(r.derived.thrust_total_n, 1) + ' N'; }],
      ['추력/중량 (T/W)', function (r) { return num(r.derived.thrust_to_weight, 3); }],
      ['호버 추력 (모터당)', function (r) { return num(r.derived.hover_per_motor_n, 2) + ' N'; }],
      ['모터 시상수', function (r) { return num(r.motor_tau_s, 3) + ' s'; }],
      ['암시 최대 RPM', function (r) { return r.derived.implied_max_rpm ? r.derived.implied_max_rpm.toLocaleString() : '—'; }],
      ['최대 상승 가속', function (r) { return num(r.derived.climb_accel_mps2, 2) + ' m/s²'; }],
      ['최대 수평 가속 (45° 틸트)', function (r) { return num(r.derived.horizontal_accel_mps2, 2) + ' m/s²'; }],
      ['롤 각가속 (대수적 상한)', function (r) { return num(r.derived.roll_alpha_radps2, 0) + ' rad/s²'; }]
    ];

    var h = '<div class="scrollx"><table><thead><tr><th>항목</th>'
          + R.map(function (r) { return '<th>' + esc(r.label) + '</th>'; }).join('')
          + '<th>비</th></tr></thead><tbody>';
    rows.forEach(function (row) {
      var vals = R.map(row[1]);
      // Flag the quantities the reference platform deliberately preserves.
      var same = vals.length === 2 && vals[0] === vals[1];
      h += '<tr><td>' + esc(row[0]) + '</td>'
         + vals.map(function (v) { return '<td><code>' + esc(v) + '</code></td>'; }).join('')
         + '<td>' + (same ? '<span class="chip chip-good">동일</span>' : '<span class="dim">—</span>') + '</td></tr>';
    });
    h += '</tbody></table></div>';
    h += '<p class="hint">' + R.map(function (r) {
      return '<code>' + esc(r.urdf) + '</code> + <code>' + esc(r.config.split('.').pop()) + '</code>';
    }).join(' · ') + ' 에서 생성. 관성은 base_link 텐서에 로터 질량의 평행축 기여를 더한 <b>조립값</b>이다.</p>';
    el.innerHTML = h;
  }

  function renderRobotNotes() {
    var el = document.getElementById('drone-notes');
    if (!el) return;
    el.innerHTML = PL.robots.map(function (r) {
      return '<div class="note"><b>' + esc(r.label) + '</b><br>' + esc(r.note) + '</div>';
    }).join('');
  }

  // ---- sensing + control ------------------------------------------------------------------
  function renderSensing() {
    var el = document.getElementById('drone-sensing');
    if (!el) return;
    var rows = [
      ['LiDAR 수평 빔', knob('NAVRL_LIDAR_HBEAMS'), 'NAVRL_LIDAR_HBEAMS'],
      ['LiDAR 수직 빔', knob('NAVRL_LIDAR_VBEAMS'), 'NAVRL_LIDAR_VBEAMS'],
      ['LiDAR 최대 거리', knob('NAVRL_LIDAR_RANGE') + ' m', 'NAVRL_LIDAR_RANGE'],
      ['장애물 토큰 수', knob('NAVRL_MAX_OBSTACLES'), 'NAVRL_MAX_OBSTACLES'],
      ['토큰 선택 FOV', knob('NAVRL_OBSTACLE_FOV_DEG') + '°', 'NAVRL_OBSTACLE_FOV_DEG'],
      ['검출기 임계값', knob('NAVRL_DETECTOR_THRESHOLD'), 'NAVRL_DETECTOR_THRESHOLD'],
      ['검출 지연', knob('NAVRL_DETECTION_LATENCY_S') + ' s', 'NAVRL_DETECTION_LATENCY_S'],
      ['최대 속도 명령', knob('NAVRL_MAX_VELOCITY') + ' m/s', 'NAVRL_MAX_VELOCITY'],
      ['최대 yaw rate', knob('NAVRL_YAW_RATE_MAX') + ' rad/s', 'NAVRL_YAW_RATE_MAX'],
      ['최대 틸트', knob('NAVRL_MAX_TILT_DEG') + '°', 'NAVRL_MAX_TILT_DEG'],
      ['속도 거버너', knob('NAVRL_SPEED_GOVERNOR'), 'NAVRL_SPEED_GOVERNOR'],
      ['거버너 제동 가속', knob('NAVRL_SPEED_GOVERNOR_BRAKE_MPS2') + ' m/s²', 'NAVRL_SPEED_GOVERNOR_BRAKE_MPS2'],
      ['거버너 반응 시간', knob('NAVRL_SPEED_GOVERNOR_REACTION_S') + ' s', 'NAVRL_SPEED_GOVERNOR_REACTION_S']
    ];
    el.innerHTML = '<div class="scrollx"><table><thead><tr><th>항목</th><th>값</th><th>knob</th></tr></thead><tbody>'
      + rows.map(function (r) {
          var p = P[r[2]];
          var badge = !p ? '' : (p.ablated ? ' <span class="chip chip-good">실험 ' + p.experiments.length + '</span>'
                                           : (p.frozen_by_contract ? ' <span class="chip chip-lock">계약 고정</span>' : ''));
          return '<tr><td>' + esc(r[0]) + '</td><td><code>' + esc(r[1]) + '</code></td>'
               + '<td><a href="parameters.html#' + esc(r[2]) + '"><code>' + esc(r[2]) + '</code></a>' + badge + '</td></tr>';
        }).join('')
      + '</tbody></table></div>';
  }

  // ---- measured envelope ------------------------------------------------------------------
  function renderEnvelope() {
    var el = document.getElementById('drone-envelope');
    if (!el) return;
    var E = PL.envelope;
    if (!E) { el.innerHTML = '<p class="hint">비행 포락선 게이트를 아직 돌리지 않았다.</p>'; return; }
    var pass = (E.checks || []).filter(function (c) { return c.pass; }).length;
    var total = (E.checks || []).length;
    var h = '<p class="criteria-lead"><b>' + esc(E.verdict) + ' ' + pass + '/' + total + '</b> · '
          + 'seed ' + esc(E.seed) + ' · ' + esc(E.num_envs) + ' envs · 막대 ' + esc(E.bars) + '개 · 거버너 off</p>';
    h += '<div class="scrollx"><table><thead><tr><th>검사</th><th>판정</th><th>상세</th></tr></thead><tbody>'
       + (E.checks || []).map(function (c) {
           return '<tr><td>' + esc(c.check) + '</td>'
                + '<td><span class="chip chip-' + (c.pass ? 'good">PASS' : 'bad">FAIL') + '</span></td>'
                + '<td class="dim">' + esc(c.detail) + '</td></tr>';
         }).join('')
       + '</tbody></table></div>';
    h += '<p class="hint">원자료 <code>' + esc(E.source) + '</code>. 이것은 <b>시뮬레이터 게이트</b>이지 '
       + '하드웨어 검증이 아니고, 항법 성능에 대해서는 아무것도 말하지 않는다.</p>';
    el.innerHTML = h;
  }

  // ---- sim vs real ------------------------------------------------------------------------
  function renderGap() {
    var el = document.getElementById('drone-gap');
    if (!el) return;
    var pay = PL.payload;
    var legacy = PL.robots[0];
    var ratioMin = pay.partial_grams_min / (legacy.mass_kg * 1000);
    var ratioMax = pay.partial_grams_max / (legacy.mass_kg * 1000);
    var h = '<div class="scrollx"><table><thead><tr><th>부품</th><th>후보</th><th>무게</th></tr></thead><tbody>'
          + pay.parts.map(function (p) {
              var grams = p.grams != null ? Number(p.grams).toFixed(p.grams % 1 ? 1 : 0) + ' g'
                : Number(p.grams_min).toFixed(1) + '–' + Number(p.grams_max).toFixed(1) + ' g';
              return '<tr><td>' + esc(p.part) + '</td><td>' + esc(p.model) + '</td><td><code>'
                   + grams + '</code>' + (p.note ? '<br><span class="dim">' + esc(p.note) + '</span>' : '')
                   + '</td></tr>';
            }).join('')
          + '<tr><td><b>불완전 부분합</b></td><td class="dim">carrier·냉각·전원·배선·mount와 기체 제외</td>'
          + '<td><b><code>' + Number(pay.partial_grams_min).toFixed(1) + '–'
          + Number(pay.partial_grams_max).toFixed(1) + ' g</code></b></td></tr>'
          + '</tbody></table></div>';
    h += '<p class="criteria-lead">아직 빠진 부품이 있는데도 이 부분합은 legacy 기체 전체 질량('
       + (legacy.mass_kg * 1000).toFixed(0) + ' g)의 <b>' + ratioMin.toFixed(2) + '–'
       + ratioMax.toFixed(2) + '배</b>다. 완성 payload나 AUW로 부르지 않는다.</p>';
    h += '<p class="hint">' + esc(pay.claim_guard || '') + '</p>';
    el.innerHTML = h;
  }

  function stamp() {
    var el = document.getElementById('drone-stamp');
    if (!el) return;
    el.innerHTML = '소스에서 생성 · <code>' + esc(PL.generator) + '</code>'
      + (PL.source_commit ? ' · <code>' + esc(PL.source_commit) + '</code>' : '');
  }

  renderAirframe();
  renderRobotNotes();
  renderSensing();
  renderEnvelope();
  renderGap();
  stamp();
})();
