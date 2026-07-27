/* MOTAR — bright daylight 3D perception arena (three.js r128).
   Dedicated stage panel (not a buried hero background).
   Bounds: x 0..24, y -12..12 (= three z), height 0..3 (= three y up). */
window.Arena = (() => {
  const X0 = 0, X1 = 24, Y0 = -12, Y1 = 12, BX0 = 3.1, BX1 = 23;
  const CAMERA_RANGE = 20, CAMERA_HALF_FOV = THREE.MathUtils.degToRad(43.5), LIDAR_RANGE = 12;
  const LIDAR_HBEAMS = 72, LIDAR_VBEAMS = 4;

  let scene, cam, renderer, controls, root, barMesh, drone, target, cameraFov, lidarLines;
  let pursuerTrail, targetTrail, targetHalo, resizeObserver;
  let groundMat, gridHelper, rimLight;
  let bars = [], playing = true, speed = 0.5, tParam = 0, goalX = 22, viewMode = 0;
  let showTrails = true, frame = 0, visible = true;
  let host, lastDrone = { x: 1, y: 0 }, trailA = [], trailB = [];
  let lastT = 0, smooth = { x: 1, y: 0 }, smoothT = { x: 22, y: 0 }, vel = { x: 1, y: 0 }, heading = 0;

  const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const coarsePointer = matchMedia('(pointer: coarse)').matches;

  function isLight() {
    const t = document.documentElement.getAttribute('data-theme');
    if (t) return t === 'light';
    return !matchMedia('(prefers-color-scheme: dark)').matches;
  }

  function rng(seed) {
    return function () {
      seed |= 0; seed = seed + 0x6D2B79F5 | 0;
      let t = Math.imul(seed ^ seed >>> 15, 1 | seed);
      t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
      return ((t ^ t >>> 14) >>> 0) / 4294967296;
    };
  }

  function placeBars(n) {
    const r = rng(12345); const pts = []; let spacing = 1.5;
    const bw = () => 0.4 + r() * 0.4;
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
    const material = new THREE.MeshStandardMaterial({ color: 0xc06a2e, roughness: .62, metalness: .08 });
    barMesh = new THREE.InstancedMesh(geometry, material, bars.length);
    barMesh.castShadow = true; barMesh.receiveShadow = true;
    const m = new THREE.Matrix4();
    bars.forEach((p, i) => {
      m.compose(new THREE.Vector3(p.x, 0, p.y), new THREE.Quaternion(), new THREE.Vector3(p.w, 1, p.w));
      barMesh.setMatrixAt(i, m);
    });
    barMesh.instanceMatrix.needsUpdate = true; root.add(barMesh);
  }

  function steer(x, y) {
    let fx = 0, fy = 0;
    for (const p of bars) {
      const dx = x - p.x, dy = y - p.y; const d = Math.hypot(dx, dy);
      if (d < 1.6 && d > 1e-3) { const w = (1.6 - d) / 1.6; fx += dx / d * w; fy += dy / d * w; }
    }
    return [fx, fy];
  }

  function clearBars(x, y, clr) {
    for (let it = 0; it < 6; it++) {
      let ex = 0, ey = 0, bad = false;
      for (const p of bars) {
        const rad = clr + p.w * 0.5;
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
    const bodyMat = new THREE.MeshStandardMaterial({ color, roughness: .28, metalness: .45 });
    const dark = new THREE.MeshStandardMaterial({ color: 0x243240, roughness: .45, metalness: .5 });
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
        new THREE.MeshBasicMaterial({ color: 0x5a7180, transparent: true, opacity: .7, side: THREE.DoubleSide }));
      rotor.rotation.x = -Math.PI / 2; rotor.position.set(x * .19 * scale, .055 * scale, z * .13 * scale); g.add(rotor);
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(.025 * scale, .03 * scale, .055 * scale, 10), dark);
      motor.position.copy(rotor.position); g.add(motor);
    }
    return g;
  }

  function makeLabel(textValue, color) {
    const canvas = document.createElement('canvas'); canvas.width = 256; canvas.height = 64;
    const ctx = canvas.getContext('2d'); ctx.clearRect(0, 0, 256, 64);
    ctx.fillStyle = 'rgba(255,255,255,.92)'; ctx.strokeStyle = color; ctx.lineWidth = 2.5;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(4, 4, 248, 56, 10); else ctx.rect(4, 4, 248, 56);
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#12202b'; ctx.font = '700 23px ui-monospace, monospace';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(textValue, 128, 33);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
      map: new THREE.CanvasTexture(canvas), transparent: true, depthTest: false,
    }));
    sprite.scale.set(1.9, .48, 1); sprite.position.set(0, 1.05, 0); sprite.renderOrder = 5;
    return sprite;
  }

  function line(points, color, opacity = 1) {
    return new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
  }

  function makeCameraFov() {
    const range = 8, half = Math.tan(CAMERA_HALF_FOV) * range;
    const verts = new Float32Array([0, .02, 0, range, .02, -half, range, .02, half,
      0, .02, 0, range, .02, half, range, .02, -half]);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    const mesh = new THREE.Mesh(geom, new THREE.MeshBasicMaterial({
      color: 0x0d8f82, transparent: true, opacity: .16, side: THREE.DoubleSide, depthWrite: false }));
    const outline = line([new THREE.Vector3(0, .03, 0), new THREE.Vector3(range, .03, -half),
      new THREE.Vector3(range, .03, half), new THREE.Vector3(0, .03, 0)], 0x0d8f82, .7);
    const group = new THREE.Group(); group.position.y = -.05; group.add(mesh, outline); return group;
  }

  function setHex(color, hex) {
    if (color && typeof color.setHex === 'function') color.setHex(hex);
  }

  function applyTheme() {
    if (!scene) return;
    const light = isLight();
    const bg = light ? 0xd5e7f2 : 0x0a1018;
    const ground = light ? 0xe8eef3 : 0x0d1a23;
    // Always replace — never assume scene.background is a Color (it starts as null).
    scene.background = new THREE.Color(bg);
    if (!scene.fog) scene.fog = new THREE.FogExp2(bg, light ? 0.012 : 0.019);
    else {
      setHex(scene.fog.color, bg);
      scene.fog.density = light ? 0.012 : 0.019;
    }
    if (groundMat) setHex(groundMat.color, ground);
    if (gridHelper) {
      const mats = Array.isArray(gridHelper.material) ? gridHelper.material : [gridHelper.material];
      setHex(mats[0] && mats[0].color, light ? 0x9bb4c4 : 0x2b4b5e);
      setHex(mats[1] && mats[1].color, light ? 0xc5d5e0 : 0x182e3c);
    }
    if (rimLight) rimLight.intensity = light ? 0.85 : 1.25;
    if (renderer) renderer.toneMappingExposure = light ? 1.15 : 1.1;
    if (lidarLines && lidarLines.material) {
      setHex(lidarLines.material.color, light ? 0x0a7f88 : 0x47d9e3);
      lidarLines.material.opacity = light ? 0.55 : 0.46;
    }
  }

  function init(el) {
    host = el;
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xd5e7f2);
    scene.fog = new THREE.FogExp2(0xd5e7f2, 0.012);

    const w = Math.max(el.clientWidth, 320);
    const h = Math.max(el.clientHeight, 320);
    cam = new THREE.PerspectiveCamera(42, w / h, .08, 180);
    cam.position.set(14, 11, 18);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.setSize(w, h, false);
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.domElement.style.display = 'block';
    renderer.domElement.style.width = '100%';
    renderer.domElement.style.height = '100%';
    el.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(cam, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = 0.06;
    controls.target.set(0, 1.1, 0);
    controls.maxPolarAngle = Math.PI / 2.05;
    controls.minDistance = 8; controls.maxDistance = 48;
    controls.enableZoom = true;
    controls.enablePan = false;
    controls.enableRotate = true;
    controls.autoRotate = !reduceMotion;
    controls.autoRotateSpeed = 0.9;
    controls.addEventListener('start', () => { controls.autoRotate = false; });

    scene.add(new THREE.HemisphereLight(0xf2f8fc, 0xb7c6d0, 0.95));
    const dl = new THREE.DirectionalLight(0xffffff, 1.55);
    dl.position.set(-10, 28, 12); dl.castShadow = true;
    dl.shadow.mapSize.set(2048, 2048);
    dl.shadow.camera.left = -25; dl.shadow.camera.right = 25;
    dl.shadow.camera.top = 25; dl.shadow.camera.bottom = -25;
    scene.add(dl);
    rimLight = new THREE.PointLight(0x2aa8a0, 0.85, 50);
    rimLight.position.set(-12, 8, -15); scene.add(rimLight);

    root = new THREE.Group(); scene.add(root);
    root.position.set(-12, 0, 0);

    groundMat = new THREE.MeshStandardMaterial({ color: 0xe8eef3, roughness: .95, metalness: .02 });
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(X1 - X0, Y1 - Y0), groundMat);
    ground.rotation.x = -Math.PI / 2; ground.position.set(12, -.02, 0);
    ground.receiveShadow = true; root.add(ground);

    gridHelper = new THREE.GridHelper(24, 24, 0x9bb4c4, 0xc5d5e0);
    gridHelper.position.set((X0 + X1) / 2, 0.01, (Y0 + Y1) / 2); root.add(gridHelper);

    root.add(line([new THREE.Vector3(0, .04, -12), new THREE.Vector3(24, .04, -12),
      new THREE.Vector3(24, .04, 12), new THREE.Vector3(0, .04, 12),
      new THREE.Vector3(0, .04, -12)], 0x5f879c, .9));

    drone = makeDrone(0x1aa86a, 0x7dffc8, 1.2); drone.add(makeLabel('PURSUER', '#1aa86a'));
    drone.position.set(1, 1, 0); root.add(drone);
    target = makeDrone(0xe04545, 0xffb0a0, 1.4); target.add(makeLabel('TARGET', '#e04545'));
    target.position.set(goalX, 1, 0); root.add(target);
    targetHalo = new THREE.Mesh(new THREE.RingGeometry(.52, .64, 40), new THREE.MeshBasicMaterial({
      color: 0xe04545, transparent: true, opacity: .95, side: THREE.DoubleSide, depthTest: false }));
    targetHalo.rotation.x = -Math.PI / 2; targetHalo.position.y = .02; target.add(targetHalo);
    cameraFov = makeCameraFov(); drone.add(cameraFov);

    const lp = new Float32Array(LIDAR_HBEAMS * LIDAR_VBEAMS * 2 * 3);
    const lg = new THREE.BufferGeometry();
    lg.setAttribute('position', new THREE.BufferAttribute(lp, 3));
    lidarLines = new THREE.LineSegments(lg, new THREE.LineBasicMaterial({
      color: 0x0a7f88, transparent: true, opacity: .55 }));
    root.add(lidarLines);
    pursuerTrail = line([], 0x178a52, .85); targetTrail = line([], 0xe04545, .7);
    root.add(pursuerTrail, targetTrail);

    makeBars(25);
    applyTheme();

    resizeObserver = new ResizeObserver(() => onResize());
    resizeObserver.observe(el);
    // layout can settle a frame later — force a second resize
    requestAnimationFrame(() => { onResize(); requestAnimationFrame(onResize); });

    addEventListener('keydown', e => { if (e.key.toLowerCase() === 'v') cycleView(); });
    new IntersectionObserver(([entry]) => {
      visible = entry.isIntersecting;
      if (visible) lastT = 0;
    }, { threshold: 0.05 }).observe(el);

    animate();
  }

  function onResize() {
    if (!host || !renderer) return;
    const w = Math.max(host.clientWidth, 1);
    const h = Math.max(host.clientHeight, 1);
    cam.aspect = w / h;
    cam.updateProjectionMatrix();
    renderer.setSize(w, h, false);
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
    const p = obj.position;
    const last = arr.length ? arr[arr.length - 1] : null;
    if (last && last.distanceToSquared(p) < 0.0144) return;
    arr.push(p.clone()); if (arr.length > 420) arr.shift();
    lineObj.geometry.dispose();
    lineObj.geometry = new THREE.BufferGeometry().setFromPoints(arr);
    lineObj.visible = showTrails;
  }

  function updateCamera() {
    if (viewMode === 0) { controls.enabled = true; return; }
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
    if (!visible || !renderer) { lastT = 0; return; }
    controls.update();

    const now = performance.now();
    const dt = Math.min((now - (lastT || now)) / 1000, 0.05); lastT = now;
    // Faster than before so motion is obvious in the dedicated stage.
    if (playing) tParam += dt * 0.14 * (1 + speed * 0.35);
    if (tParam > 1) {
      tParam = 0;
      trailA.length = 0; trailB.length = 0;
    }

    const x = 1 + tParam * (goalX - 1);
    let y = Math.sin(tParam * Math.PI * 3) * 4;
    const [, sy] = steer(x, y); y += sy * 2.2;
    const [px, py] = clearBars(x, y, 0.2);
    const k = 1 - Math.exp(-dt * 9);
    smooth.x += (px - smooth.x) * k; smooth.y += (py - smooth.y) * k;
    const dx = smooth.x, dy = smooth.y;

    vel.x += ((dx - lastDrone.x) / Math.max(dt, 1e-3) - vel.x) * (1 - Math.exp(-dt * 6));
    vel.y += ((dy - lastDrone.y) / Math.max(dt, 1e-3) - vel.y) * (1 - Math.exp(-dt * 6));
    if (Math.hypot(vel.x, vel.y) > 0.05) {
      const want = Math.atan2(vel.y, vel.x);
      const d = Math.atan2(Math.sin(want - heading), Math.cos(want - heading));
      heading += d * (1 - Math.exp(-dt * 7));
    }
    drone.position.set(dx, 1 + .03 * Math.sin(tParam * 30), dy);
    drone.rotation.y = -heading;
    const bank = THREE.MathUtils.clamp(-vel.y * 0.12, -.28, .28);
    drone.rotation.z += (bank - drone.rotation.z) * (1 - Math.exp(-dt * 8));
    // Spin rotors for visible motion even when path is slow.
    drone.children.forEach(ch => {
      if (ch.geometry && ch.geometry.type === 'RingGeometry') ch.rotation.z += dt * 18;
    });
    lastDrone = { x: dx, y: dy };

    let tx = goalX, ty = (speed > 0) ? Math.sin(tParam * Math.PI * 2 * (0.55 + speed * 0.15)) * 8 : 0;
    [tx, ty] = clearBars(tx, ty, 0.5);
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
    if (targetHalo && targetHalo.material) {
      setHex(targetHalo.material.color, vis.visible ? 0x1aa86a : 0xe04545);
      targetHalo.material.opacity = vis.visible ? .95 : .55;
      targetHalo.scale.setScalar(1 + .14 * Math.sin(frame * .1));
    }
    if (cameraFov) {
      cameraFov.children.forEach(o => {
        const mats = o.material == null ? [] : (Array.isArray(o.material) ? o.material : [o.material]);
        mats.forEach(m => setHex(m && m.color, vis.visible ? 0x0d8f82 : 0xe04545));
      });
    }
    drawLidar(dx, dy);
    updateTrail(drone, trailA, pursuerTrail);
    updateTrail(target, trailB, targetTrail);
    updateCamera(); frame++;
    renderer.render(scene, cam);
  }

  function cycleView() {
    viewMode = (viewMode + 1) % 3;
    if (viewMode === 0) {
      cam.position.set(14, 11, 18);
      controls.target.set(0, 1.1, 0);
      controls.enabled = true;
      controls.update();
    }
    const btn = document.getElementById('btn-view');
    if (btn) btn.textContent = ['시점 · overview', '시점 · chase', '시점 · sensor'][viewMode];
    return viewMode;
  }

  return {
    init,
    setBars(n) { makeBars(n); trailA.length = 0; trailB.length = 0; },
    setSpeed(s) { speed = s; },
    setPlaying(p) { playing = p; },
    setLidar(v) { lidarLines.visible = v; },
    setCamera(v) { cameraFov.visible = v; },
    setTrails(v) { showTrails = v; pursuerTrail.visible = v; targetTrail.visible = v; },
    cycleView,
    recolor: applyTheme,
  };
})();
