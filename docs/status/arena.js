/* MOTAR — 3D perception arena (three.js r128).
   Arena bounds: x 0..24, y -12..12 (= three z), height 0..3 (= three y up).

   The arena is the page's hero background, which drives three interaction rules:
     - wheel zoom and pan are OFF, so the wheel always scrolls the page
     - on coarse pointers rotation is OFF too, so a swipe scrolls instead of orbiting
     - rendering stops while the hero is off screen
*/
window.Arena = (() => {
  const X0 = 0, X1 = 24, Y0 = -12, Y1 = 12, BX0 = 3.1, BX1 = 23;
  // Sensor spec mirrors the policy being trained (FOV-240 representation, 2026-07-27):
  // 72 x 4 LiDAR beams at 12 m, forward RGB-D detector at 87 deg / 20 m.
  const CAMERA_RANGE = 20, CAMERA_HALF_FOV = THREE.MathUtils.degToRad(43.5), LIDAR_RANGE = 12;
  const LIDAR_HBEAMS = 72, LIDAR_VBEAMS = 4;

  let scene, cam, renderer, controls, root, barMesh, drone, target, cameraFov, lidarLines;
  let pursuerTrail, targetTrail, targetHalo, resizeObserver;
  let bars = [], playing = true, speed = 0, tParam = 0, goalX = 22, viewMode = 0;
  let showTrails = true, frame = 0, visible = true;
  let host, lastDrone = { x: 1, y: 0 }, trailA = [], trailB = [];
  // Smoothing state for the render loop (see animate()). `smooth`/`smoothT` are the drawn
  // positions that chase the hard bar-clearance projection; `vel` and `heading` are filtered
  // so the airframe never snaps when that projection steps.
  let lastT = 0, smooth = { x: 1, y: 0 }, smoothT = { x: 22, y: 0 }, vel = { x: 1, y: 0 }, heading = 0;

  const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const coarsePointer = matchMedia('(pointer: coarse)').matches;

  // seeded RNG (mulberry32) so a given bar count is reproducible/stable
  function rng(seed) {
    return function () {
      seed |= 0; seed = seed + 0x6D2B79F5 | 0;
      let t = Math.imul(seed ^ seed >>> 15, 1 | seed);
      t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
      return ((t ^ t >>> 14) >>> 0) / 4294967296;
    };
  }

  // random-rejection placement mirroring asset_manager: min spacing with saturation
  // relaxation (x0.8 every N failures) so high counts still fill the band.
  function placeBars(n) {
    const r = rng(12345); const pts = []; let spacing = 1.5;
    const bw = () => 0.4 + r() * 0.4;               // footprint 0.4..0.8
    let fails = 0, guard = 0;
    while (pts.length < n && guard < n * 400) {
      guard++;
      const x = BX0 + r() * (BX1 - BX0), y = Y0 + 0.6 + r() * ((Y1 - Y0) - 1.2);
      let ok = true;
      for (const p of pts) { const dx = x - p.x, dy = y - p.y; if (dx * dx + dy * dy < spacing * spacing) { ok = false; break; } }
      if (ok) { pts.push({ x, y, w: bw() }); fails = 0; }
      else { fails++; if (fails >= 128) { spacing *= 0.8; fails = 0; } }
    }
    return pts;
  }

  function makeBars(n) {
    if (barMesh) { root.remove(barMesh); barMesh.geometry.dispose(); barMesh.material.dispose(); }
    bars = placeBars(n);
    const geometry = new THREE.BoxGeometry(1, 2, 1);
    geometry.translate(0, 1, 0);
    const material = new THREE.MeshStandardMaterial({ color: 0xb8733d, roughness: .7, metalness: .06 });
    barMesh = new THREE.InstancedMesh(geometry, material, bars.length);
    barMesh.castShadow = true; barMesh.receiveShadow = true;
    const m = new THREE.Matrix4();
    bars.forEach((p, i) => {
      m.compose(new THREE.Vector3(p.x, 0, p.y), new THREE.Quaternion(), new THREE.Vector3(p.w, 1, p.w));
      barMesh.setMatrixAt(i, m);
    });
    barMesh.instanceMatrix.needsUpdate = true; root.add(barMesh);
  }

  // nearest-bar avoidance so the drone visually weaves rather than clips
  function steer(x, y) {
    let fx = 0, fy = 0;
    for (const p of bars) {
      const dx = x - p.x, dy = y - p.y; const d = Math.hypot(dx, dy);
      if (d < 1.6 && d > 1e-3) { const w = (1.6 - d) / 1.6; fx += dx / d * w; fy += dy / d * w; }
    }
    return [fx, fy];
  }

  // Push a point out of every bar's clearance disc (composite escape), mirroring the real
  // _advance_target clearance push-out so the target never sits inside an obstacle. Iterated a
  // few times to resolve overlapping discs at high density.
  function clearBars(x, y, clr) {
    for (let it = 0; it < 6; it++) {
      let ex = 0, ey = 0, bad = false;
      for (const p of bars) {
        const rad = clr + p.w * 0.5;            // bar half-footprint + capture clearance
        const dx = x - p.x, dy = y - p.y; const d = Math.hypot(dx, dy);
        if (d < rad) { bad = true; const inv = d > 1e-3 ? 1 / d : 1; ex += dx * inv; ey += dy * inv; }
      }
      if (!bad) break;
      const en = Math.hypot(ex, ey) || 1; x += ex / en * 0.25; y += ey / en * 0.25;
    }
    return [x, y];
  }

  function makeDrone(color, accent, scale = 1) {
    const g = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color, roughness: .32, metalness: .48 });
    const dark = new THREE.MeshStandardMaterial({ color: 0x16232c, roughness: .5, metalness: .55 });
    const glow = new THREE.MeshBasicMaterial({ color: accent });
    const body = new THREE.Mesh(new THREE.BoxGeometry(.36 * scale, .11 * scale, .24 * scale), bodyMat);
    body.castShadow = true; g.add(body);
    const nose = new THREE.Mesh(new THREE.ConeGeometry(.09 * scale, .20 * scale, 10), glow);
    nose.rotation.z = -Math.PI / 2; nose.position.x = .26 * scale; g.add(nose);
    for (const z of [-1, 1]) {
      const arm = new THREE.Mesh(new THREE.BoxGeometry(.46 * scale, .035 * scale, .045 * scale), dark);
      arm.rotation.y = z * .58; g.add(arm);
    }
    for (const x of [-1, 1]) for (const z of [-1, 1]) {
      const rotor = new THREE.Mesh(new THREE.RingGeometry(.095 * scale, .12 * scale, 28),
        new THREE.MeshBasicMaterial({ color: 0xbcd5df, transparent: true, opacity: .55, side: THREE.DoubleSide }));
      rotor.rotation.x = -Math.PI / 2; rotor.position.set(x * .19 * scale, .055 * scale, z * .13 * scale); g.add(rotor);
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(.025 * scale, .03 * scale, .055 * scale, 10), dark);
      motor.position.copy(rotor.position); g.add(motor);
    }
    return g;
  }

  function makeLabel(textValue, color) {
    const canvas = document.createElement('canvas'); canvas.width = 256; canvas.height = 64;
    const ctx = canvas.getContext('2d'); ctx.clearRect(0, 0, 256, 64);
    ctx.fillStyle = 'rgba(4,12,18,.82)'; ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(4, 4, 248, 56, 10); else ctx.rect(4, 4, 248, 56);
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#f4fbff'; ctx.font = '600 23px ui-monospace, monospace';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(textValue, 128, 33);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
      map: new THREE.CanvasTexture(canvas), transparent: true, depthTest: false,
    }));
    sprite.scale.set(1.9, .48, 1); sprite.position.set(0, 1.0, 0); sprite.renderOrder = 5;
    return sprite;
  }

  function line(points, color, opacity = 1) {
    return new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
  }

  function makeCameraFov() {
    const range = 7, half = Math.tan(CAMERA_HALF_FOV) * range;
    const verts = new Float32Array([0, .02, 0, range, .02, -half, range, .02, half,
      0, .02, 0, range, .02, half, range, .02, -half]);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    const mesh = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({
      color: 0x31d6e2, transparent: true, opacity: .12, side: THREE.DoubleSide, depthWrite: false }));
    const outline = line([new THREE.Vector3(0, .03, 0), new THREE.Vector3(range, .03, -half),
      new THREE.Vector3(range, .03, half), new THREE.Vector3(0, .03, 0)], 0x56e4ed, .48);
    const group = new THREE.Group(); group.position.y = -.05; group.add(mesh, outline); return group;
  }

  function init(el) {
    host = el;
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x05080b);
    scene.fog = new THREE.FogExp2(0x05080b, .019);
    cam = new THREE.PerspectiveCamera(46, el.clientWidth / el.clientHeight, .08, 180);
    cam.position.set(17, 9.5, 21);
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.75));
    renderer.setSize(el.clientWidth, el.clientHeight);
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.12;
    el.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(cam, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = 0.055;
    controls.target.set(0, 1.2, 0);
    controls.maxPolarAngle = Math.PI / 2.06;
    controls.minDistance = 9; controls.maxDistance = 52;
    // The hero fills the viewport, so the wheel and two-finger gestures must keep
    // scrolling the page rather than zooming the scene.
    controls.enableZoom = false;
    controls.enablePan = false;
    // A swipe on a touch screen has to scroll, so orbiting is mouse-only.
    controls.enableRotate = !coarsePointer;
    // Slow drift gives the hero the presence of a background video; it yields
    // permanently the moment the visitor takes the camera.
    controls.autoRotate = !reduceMotion && !coarsePointer;
    controls.autoRotateSpeed = 0.22;
    controls.addEventListener('start', () => { controls.autoRotate = false; });

    scene.add(new THREE.HemisphereLight(0xaad8ee, 0x13202b, .72));
    const dl = new THREE.DirectionalLight(0xf4fbff, 2.2);
    dl.position.set(-8, 26, 14); dl.castShadow = true;
    dl.shadow.mapSize.set(2048, 2048);
    dl.shadow.camera.left = -25; dl.shadow.camera.right = 25;
    dl.shadow.camera.top = 25; dl.shadow.camera.bottom = -25;
    scene.add(dl);
    const rim = new THREE.PointLight(0x20d4df, 1.25, 42); rim.position.set(-12, 8, -15); scene.add(rim);

    root = new THREE.Group(); scene.add(root);
    root.position.set(-12, 0, 0);

    const ground = new THREE.Mesh(new THREE.PlaneGeometry(X1 - X0, Y1 - Y0),
      new THREE.MeshStandardMaterial({ color: 0x0d1a23, roughness: .92, metalness: .03 }));
    ground.rotation.x = -Math.PI / 2; ground.position.set(12, -.02, 0);
    ground.receiveShadow = true; root.add(ground);
    const gridHelper = new THREE.GridHelper(24, 24, 0x2b4b5e, 0x182e3c);
    gridHelper.position.set((X0 + X1) / 2, 0.01, (Y0 + Y1) / 2); root.add(gridHelper);

    root.add(line([new THREE.Vector3(0, .04, -12), new THREE.Vector3(24, .04, -12),
      new THREE.Vector3(24, .04, 12), new THREE.Vector3(0, .04, 12),
      new THREE.Vector3(0, .04, -12)], 0x4b768b, .8));
    const altGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 1, -12), new THREE.Vector3(24, 1, -12), new THREE.Vector3(24, 1, 12),
      new THREE.Vector3(0, 1, 12), new THREE.Vector3(0, 1, -12)]);
    const altLine = new THREE.Line(altGeom, new THREE.LineDashedMaterial({
      color: 0x24bbc6, dashSize: .35, gapSize: .22, transparent: true, opacity: .22 }));
    altLine.computeLineDistances(); root.add(altLine);

    drone = makeDrone(0x3bd68d, 0xc6ffec, 1.15); drone.add(makeLabel('PURSUER', '#3bd68d'));
    drone.position.set(1, 1, 0); root.add(drone);
    target = makeDrone(0xf05252, 0xffc1a8, 1.35); target.add(makeLabel('TARGET', '#ff685f'));
    target.position.set(goalX, 1, 0); root.add(target);
    targetHalo = new THREE.Mesh(new THREE.RingGeometry(.52, .64, 40), new THREE.MeshBasicMaterial({
      color: 0xff665e, transparent: true, opacity: .9, side: THREE.DoubleSide, depthTest: false }));
    targetHalo.rotation.x = -Math.PI / 2; targetHalo.position.y = .01; target.add(targetHalo);
    cameraFov = makeCameraFov(); drone.add(cameraFov);

    const lp = new Float32Array(LIDAR_HBEAMS * LIDAR_VBEAMS * 2 * 3);
    const lg = new THREE.BufferGeometry();
    lg.setAttribute('position', new THREE.BufferAttribute(lp, 3));
    lidarLines = new THREE.LineSegments(lg, new THREE.LineBasicMaterial({
      color: 0x47d9e3, transparent: true, opacity: .46 }));
    root.add(lidarLines);
    pursuerTrail = line([], 0x8ef6c7, .75); targetTrail = line([], 0xff8a76, .62);
    root.add(pursuerTrail, targetTrail);

    makeBars(25);
    resizeObserver = new ResizeObserver(onResize); resizeObserver.observe(el);
    addEventListener('keydown', e => { if (e.key.toLowerCase() === 'v') cycleView(); });
    // Stop drawing entirely once the hero has scrolled away.
    new IntersectionObserver(([entry]) => { visible = entry.isIntersecting; }, { threshold: 0 }).observe(el);
    animate();
  }

  function onResize() {
    if (!host || !host.clientWidth) return;
    cam.aspect = host.clientWidth / host.clientHeight;
    cam.updateProjectionMatrix();
    renderer.setSize(host.clientWidth, host.clientHeight);
  }

  function rayHit(x, y, a, maxRange) {
    let best = maxRange; const ux = Math.cos(a), uy = Math.sin(a);
    for (const p of bars) {
      const ox = p.x - x, oy = p.y - y, t = ox * ux + oy * uy;
      if (t <= 0 || t >= best) continue;
      const d2 = ox * ox + oy * oy - t * t, r = p.w * .72;
      if (d2 < r * r) { const hit = t - Math.sqrt(r * r - d2); if (hit > 0) best = Math.min(best, hit); }
    }
    return best;
  }

  function drawLidar(x, y) {
    if (!lidarLines.visible) return;
    const a = lidarLines.geometry.attributes.position.array; let k = 0;
    for (let layer = 0; layer < LIDAR_VBEAMS; layer++) for (let i = 0; i < LIDAR_HBEAMS; i++) {
      const ang = i / LIDAR_HBEAMS * Math.PI * 2, hit = rayHit(x, y, ang, LIDAR_RANGE), h = .82 + layer * .12;
      a[k++] = x; a[k++] = h; a[k++] = y;
      a[k++] = x + Math.cos(ang) * hit; a[k++] = h; a[k++] = y + Math.sin(ang) * hit;
    }
    lidarLines.geometry.attributes.position.needsUpdate = true;
  }

  function visibility(dx, dy, tx, ty, hdg) {
    const vx = tx - dx, vy = ty - dy, range = Math.hypot(vx, vy), bearing = Math.atan2(vy, vx);
    const rel = Math.atan2(Math.sin(bearing - hdg), Math.cos(bearing - hdg));
    const inFov = Math.abs(rel) <= CAMERA_HALF_FOV && range <= CAMERA_RANGE;
    const hit = rayHit(dx, dy, bearing, Math.min(range, CAMERA_RANGE));
    return { range, visible: inFov && hit >= range - .28, occluded: inFov && hit < range - .28, inFov };
  }

  function updateTrail(obj, arr, lineObj) {
    // Sample by DISTANCE, not every Nth frame. Frame sampling made the trail's vertex spacing
    // depend on speed and refresh rate, so it rendered as a visibly polygonal chain that
    // lurched forward a segment at a time; a distance threshold gives even, stable spacing.
    const p = obj.position;
    const last = arr.length ? arr[arr.length - 1] : null;
    if (last && last.distanceToSquared(p) < 0.0144) return;   // 0.12 m
    arr.push(p.clone()); if (arr.length > 420) arr.shift();
    lineObj.geometry.dispose();
    lineObj.geometry = new THREE.BufferGeometry().setFromPoints(arr);
    lineObj.visible = showTrails;
  }

  function updateCamera() {
    if (viewMode === 0) { controls.enabled = !coarsePointer; return; }
    controls.enabled = false;
    const wp = new THREE.Vector3(); drone.getWorldPosition(wp);
    const dir = new THREE.Vector3(1, 0, 0).applyQuaternion(drone.getWorldQuaternion(new THREE.Quaternion()));
    if (viewMode === 1) {
      cam.position.copy(wp).addScaledVector(dir, -3.4).add(new THREE.Vector3(0, 2.1, 0));
      cam.lookAt(wp.clone().addScaledVector(dir, 4));
    } else {
      cam.position.copy(wp).add(new THREE.Vector3(0, .12, 0)).addScaledVector(dir, .24);
      cam.lookAt(wp.clone().addScaledVector(dir, 8));
    }
  }

  function animate() {
    requestAnimationFrame(animate);
    if (!visible) { lastT = 0; return; }   // hero off screen: draw nothing
    controls.update();

    // --- time-based, not frame-based -------------------------------------------------
    // tParam used to advance by a fixed amount PER FRAME, so playback speed tracked the
    // refresh rate and every dropped frame became a visible jump. Advance by elapsed time
    // instead, clamping dt so returning to a backgrounded tab does not teleport the drone.
    const now = performance.now();
    const dt = Math.min((now - (lastT || now)) / 1000, 0.05); lastT = now;
    if (playing) tParam += dt * 0.096 * (1 + speed * 0.25);
    if (tParam > 1) tParam = 0;

    // Nominal path: sweep x 1..goalX; y follows a gentle weave plus bar repulsion.
    const x = 1 + tParam * (goalX - 1);
    let y = Math.sin(tParam * Math.PI * 3) * 4;
    const [, sy] = steer(x, y); y += sy * 2.2;
    // clearBars() is a HARD projection: it iteratively shoves the point out of any bar it
    // overlaps, in fixed 0.25 steps. Near a bar the correction can switch on and reverse
    // direction between consecutive frames, so the projected point is not a continuous
    // function of tParam -- that discontinuity was the "ticking" jump. Feed it through an
    // exponential smoother so the rendered body follows the projection instead of snapping
    // to it. Frame-rate independent because the coefficient is derived from dt.
    const [px, py] = clearBars(x, y, 0.2);
    const k = 1 - Math.exp(-dt * 9);
    smooth.x += (px - smooth.x) * k; smooth.y += (py - smooth.y) * k;
    const dx = smooth.x, dy = smooth.y;

    // Heading came from the frame-to-frame delta of the *projected* point, so every
    // projection jump snapped the whole airframe around. Track a smoothed velocity and
    // turn toward it through the shortest angle, at a bounded rate.
    vel.x += ((dx - lastDrone.x) / Math.max(dt, 1e-3) - vel.x) * (1 - Math.exp(-dt * 6));
    vel.y += ((dy - lastDrone.y) / Math.max(dt, 1e-3) - vel.y) * (1 - Math.exp(-dt * 6));
    if (Math.hypot(vel.x, vel.y) > 0.05) {
      const want = Math.atan2(vel.y, vel.x);
      const d = Math.atan2(Math.sin(want - heading), Math.cos(want - heading));
      heading += d * (1 - Math.exp(-dt * 7));
    }
    drone.position.set(dx, 1 + .025 * Math.sin(tParam * 30), dy);
    drone.rotation.y = -heading;
    // Bank from the smoothed lateral velocity rather than a raw per-frame difference.
    const bank = THREE.MathUtils.clamp(-vel.y * 0.12, -.28, .28);
    drone.rotation.z += (bank - drone.rotation.z) * (1 - Math.exp(-dt * 8));
    lastDrone = { x: dx, y: dy };

    // target: static (speed 0) sits at the goal; a moving target bounces in y BUT is pushed out
    // of bar clearance every frame -- exactly like the sim, so it never passes through a bar.
    let tx = goalX, ty = (speed > 0) ? Math.sin(tParam * Math.PI * 2 * (0.5 + speed * 0.1)) * 8 : 0;
    [tx, ty] = clearBars(tx, ty, 0.5);
    // Same smoothing for the target, whose projection jumps for the same reason.
    smoothT.x += (tx - smoothT.x) * k; smoothT.y += (ty - smoothT.y) * k;
    tx = smoothT.x; ty = smoothT.y;
    target.position.set(tx, 1 + .04 * Math.sin(tParam * 24), ty);
    target.rotation.y = Math.sin(tParam * Math.PI * 2) * .5;

    const vis = visibility(dx, dy, tx, ty, heading);
    const state = document.getElementById('hud-camera');
    if (state) {
      state.textContent = vis.visible ? 'DETECTED' : vis.occluded ? 'OCCLUDED' : 'OUT OF FOV';
      state.className = vis.visible ? 'seen' : 'lost';
    }
    const rangeEl = document.getElementById('hud-range');
    if (rangeEl) rangeEl.textContent = vis.range.toFixed(1) + ' m';
    targetHalo.material.color.setHex(vis.visible ? 0x5bf2a6 : 0xff665e);
    targetHalo.material.opacity = vis.visible ? .9 : .5;
    targetHalo.scale.setScalar(1 + .12 * Math.sin(frame * .08));
    cameraFov.children.forEach(o => { if (o.material) o.material.color.setHex(vis.visible ? 0x31d6e2 : 0xf06652); });
    drawLidar(dx, dy);
    updateTrail(drone, trailA, pursuerTrail);
    updateTrail(target, trailB, targetTrail);
    updateCamera(); frame++;
    renderer.render(scene, cam);
  }

  function cycleView() {
    viewMode = (viewMode + 1) % 3;
    if (viewMode === 0) {
      cam.position.set(17, 9.5, 21);
      controls.target.set(0, 1.2, 0);
      controls.enabled = !coarsePointer;
      controls.update();
    }
    const btn = document.getElementById('btn-view');
    if (btn) btn.textContent = ['시점 · overview', '시점 · chase', '시점 · sensor'][viewMode];
    return viewMode;
  }

  return {
    init,
    setBars(n) { makeBars(n); },
    setSpeed(s) { speed = s; },
    setPlaying(p) { playing = p; },
    setLidar(v) { lidarLines.visible = v; },
    setCamera(v) { cameraFov.visible = v; },
    setTrails(v) { showTrails = v; pursuerTrail.visible = v; targetTrail.visible = v; },
    cycleView,
    recolor() {
      if (!renderer) return;
      const light = document.documentElement.dataset.theme === 'light';
      renderer.toneMappingExposure = light ? 1.24 : 1.1;
      const bg = light ? 0x0a141b : 0x05080b;
      scene.background.setHex(bg); scene.fog.color.setHex(bg);
    },
  };
})();
