(function () {
  'use strict';

  const geometry = {
    arena_xy_m: 40,
    goal_dist_m: [6, 28],
    target_speed_m: [0.3, 1.5],
    bar_x_min_ratio: 0,
    bar_x_max_ratio: 1,
    bar_height_m: 3,
    placement_mode: 'navrl_band',
    placement_touch_m: 0.4,
    placement_gap_m: 1.6,
  };

  function byId(id) { return document.getElementById(id); }

  function boot() {
    const stage = byId('stage');
    if (!stage) return;
    try {
      if (typeof THREE === 'undefined') throw new Error('THREE is unavailable');
      if (!THREE.OrbitControls) throw new Error('OrbitControls is unavailable');
      if (!window.Arena) throw new Error('Arena module is unavailable');

      stage.textContent = '';
      window.Arena.configure(geometry);
      window.Arena.init(stage);

      const bars = byId('sl-bars');
      const speed = byId('sl-speed');
      const setBars = function () {
        const value = Number(bars.value);
        byId('lbl-bars').textContent = String(value);
        byId('hud-bars').textContent = String(value);
        byId('hud-density').textContent = (value / 16).toFixed(1);
        window.Arena.setBars(value);
      };
      const setSpeed = function () {
        const value = Number(speed.value) / 10;
        byId('lbl-speed').textContent = value.toFixed(1);
        window.Arena.setSpeed(value);
      };

      bars.addEventListener('input', setBars);
      speed.addEventListener('input', setSpeed);

      let playing = true;
      byId('btn-play').addEventListener('click', function () {
        playing = !playing;
        window.Arena.setPlaying(playing);
        this.textContent = playing ? 'Pause' : 'Play';
        this.setAttribute('aria-pressed', String(playing));
      });
      byId('btn-view').addEventListener('click', function () { window.Arena.cycleView(); });
      byId('btn-preset').addEventListener('click', function () {
        bars.value = '205';
        setBars();
      });
      byId('cb-camera').addEventListener('change', function () { window.Arena.setCamera(this.checked); });
      byId('cb-lidar').addEventListener('change', function () { window.Arena.setLidar(this.checked); });
      byId('cb-trails').addEventListener('change', function () { window.Arena.setTrails(this.checked); });

      setBars();
      setSpeed();
    } catch (error) {
      stage.innerHTML = '<p class="viewer-error">3D viewer could not start: '
        + String(error && error.message ? error.message : error) + '</p>';
    }
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
}());
