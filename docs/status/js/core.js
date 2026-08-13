/* Shared helpers and the render registry for every MOTAR dashboard page.
 *
 * Split out of the old single-page app.js on 2026-08-13. Each page loads core.js, then only the
 * panels-*.js it actually needs; every panels file ends with MOTAR.register(...), and core boots
 * once on DOMContentLoaded.
 *
 * No bundler: these are classic scripts, so the browser guarantees they execute in document order
 * and every register() has run before boot(). Do NOT add defer or type=module to any of them --
 * both imply deferral and would silently reorder against status.fallback.js.
 */

const pct = x => x == null ? '—' : (x * 100).toFixed(1) + '%';

const finite = x => (x == null || x === '' ? null : (Number.isFinite(Number(x)) ? Number(x) : null));
// Obstacle-placement area [m^2]. v1: a 24 m arena with bars confined to x in 0.13..0.96 -> 478.
// v2: a 40 m arena with the band widened to the full width -> the whole 1600 m^2 footprint.
// Set from status.json so a v1 and a v2 run are never reported on the same denominator.

// Obstacle-placement area [m^2]. v1: a 24 m arena with bars confined to x in 0.13..0.96 -> 478.
// v2: a 40 m arena with the band widened to the full width -> the whole 1600 m^2 footprint.
// Set from status.json so a v1 and a v2 run are never reported on the same denominator.
let BAND_AREA = 478;
// Density per 100 m2. The area is per SERIES, not per page: an archived v1 curve (478 m2 band in
// a 24 m arena) rendered against the live v2 denominator (1600 m2) understated its density 3.3x
// -- 25 bars read as 1.6/100m2 instead of 5.2. Callers drawing an archived curve pass that
// curve's own placement_area_m2; the arena HUD, which describes the live task, passes nothing.

// Density per 100 m2. The area is per SERIES, not per page: an archived v1 curve (478 m2 band in
// a 24 m arena) rendered against the live v2 denominator (1600 m2) understated its density 3.3x
// -- 25 bars read as 1.6/100m2 instead of 5.2. Callers drawing an archived curve pass that
// curve's own placement_area_m2; the arena HUD, which describes the live task, passes nothing.
const perc100 = (n, area) => ((n / (area || BAND_AREA)) * 100).toFixed(1);
// Geometry of the task actually being described; overwritten from status.json.arena_geometry.

// Geometry of the task actually being described; overwritten from status.json.arena_geometry.
let ARENA_GEO = null;

/* Retarget the arena panel (3D scene, bars slider, caption) at the running task's real geometry.
   Without this the panel silently drew the v1 24 m arena while a 40 m v2 run was training. */

const cssv = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;

// Instant-paint copy, assigned by status.fallback.js (a classic script loaded just before this
// one, so it is guaranteed to have run). Returns null when absent -- the page then paints from
// fetchStatus() alone, which is the normal path over http.
function fallbackStatus() {
  return window.__STATUS_FALLBACK__ || null;
}

async function fetchStatus() {
  const r = await fetch('status.json', { cache: 'no-store' });
  if (!r.ok) throw new Error('http ' + r.status);
  return await r.json();
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

/* ---- render registry ------------------------------------------------------------------- */
window.MOTAR = {
  _renderers: [],
  _hooks: [],
  /** Called by each panels-*.js with the renderers that page owns. */
  register: function () { this._renderers.push.apply(this._renderers, arguments); },
  /** Side effects that need the snapshot before the renderers run (setup page arena geometry). */
  hook: function (fn) { this._hooks.push(fn); },

  onStatus: function (s) {
    if (s && s.placement_area_m2) BAND_AREA = Number(s.placement_area_m2);
    if (s && s.arena_geometry) ARENA_GEO = s.arena_geometry;
    this._hooks.forEach(function (fn) {
      try { fn(s); } catch (e) { console.error('hook', e); }
    });
  },

  run: function (s) {
    if (!s) return;
    MOTAR.onStatus(s);
    if (window.renderShellPill) window.renderShellPill(s);
    // Per-renderer isolation: the pre-split renderAll ran them all in one try block, so the first
    // throw silently killed every panel below it.
    MOTAR._renderers.forEach(function (fn) {
      try { fn(s); } catch (e) { console.error(fn.name || 'renderer', e); }
    });
  },

  markSnapshot: function () {
    var pill = document.getElementById('livepill');
    if (pill) pill.className = 'pill is-snapshot';
    var t = document.getElementById('livepill-txt');
    if (t) t.textContent = 'SNAP';
    var ctx = document.getElementById('top-context');
    if (ctx) ctx.textContent = 'v2 \u00b7 FALLBACK \u00b7 offline snapshot';
    var fresh = document.getElementById('live-freshness');
    if (fresh) {
      fresh.textContent = 'fallback snapshot \u00b7 live fetch failed';
      fresh.className = 'snapshot';
    }
  },

  boot: async function () {
    MOTAR.run(fallbackStatus());            // instant paint from status.fallback.js
    try { MOTAR.run(await fetchStatus()); } // authoritative pass
    catch (e) { MOTAR.markSnapshot(); }
    MOTAR._ready = true;
    (MOTAR._afterBoot || []).forEach(function (fn) { try { fn(); } catch (e) { console.error(e); } });
  },

  /** Run fn once the first render pass has completed (the 3D arena needs ARENA_GEO first). */
  afterBoot: function (fn) {
    (this._afterBoot = this._afterBoot || []).push(fn);
    if (this._ready) fn();
  }
};

document.addEventListener('DOMContentLoaded', function () { MOTAR.boot(); });
