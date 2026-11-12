"""S4: Gazebo's own sensors in place of the bridge's ray casts (plan stage S4, docs/plans/sitl_validation_plan_v1.md).

Pieces used by the S4 runner:
  - S4Client: the TCP client of the world plugin tools/sitl_s4/motar_s4_bridge.cc (target pose in, frames out);
  - motar_lidar / motar_rgbd: Gazebo frames converted to the arrays MOTAR's perception takes
    (LiDAR (1, 288) metres in MOTAR's beam order and range limits; RGB (1, 3, 90, 160) in [0, 1]; depth (1, 90, 160)
    metres along each pixel's ray, as MOTAR's renderer measures, clamped to the 20 m camera range);
  - world_text_s4: the S3b world plus the kinematic, collision-free target sphere and the plugin;
  - GazeboSensorEpisode: tools/sitl_bridge.Episode fed with the Gazebo arrays, with MOTAR's own render at the same
    pose computed beside it as a shadow (never fed to perception) for the sensor-gap record.
"""
import contextlib
import math
import socket
import struct
import time

import numpy as np

H, W = 4, 72                          # MOTAR LiDAR rows (elevation +20 .. -10 deg) and columns (azimuth +180 .. -175)
LIDAR_MAX, LIDAR_MIN = 12.0, 0.2
CAM_W, CAM_H = 160, 90
CAM_HFOV = math.radians(87.0)
CAM_FAR = 20.0
PORT = 4590
TARGET_MODEL = "motar_target"
TARGET_RADIUS = 0.15                  # the camera target sphere (navrl_task_config camera_target_radius)
TARGET_RGB = (0.88, 0.08, 0.045)      # navrl_detector nominal target colour
SENSOR_SCOPE = "default::navrl_quad_s4::base_link::"
HDR = struct.Struct("<4sI5d7I")


class ReadinessError(RuntimeError):
    pass


class ReadinessTimeout(ReadinessError, TimeoutError):
    pass


class S4Client:
    def __init__(self, port=PORT, timeout=5.0):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._budget = None

    @contextlib.contextmanager
    def bounded_io(self, deadline, service, clock=time.monotonic, quantum_s=0.025):
        """One absolute I/O budget; retain partial reads across service slices.

        The callback must itself be bounded. This is cooperative socket scheduling,
        not a hard real-time guarantee against host/process stalls.
        A failed transaction closes the stream; partial replies cannot be reused.
        """
        if self._budget is not None or not math.isfinite(deadline) or not 0 < quantum_s <= 0.05:
            raise ValueError("invalid/nested S4 I/O budget")
        previous = self.s.gettimeout()
        self._budget = (deadline, service, clock, quantum_s)
        try:
            self._service_io()
            yield
            self._service_io()
        except BaseException:
            self.close()
            raise
        else:
            self.s.settimeout(previous)
        finally:
            self._budget = None

    def _service_io(self):
        if self._budget is None:
            return
        deadline, service, clock, quantum = self._budget
        if clock() >= deadline:
            raise ReadinessTimeout("HARNESS_INVALID initial target I/O deadline")
        service()
        remaining = deadline - clock()
        if remaining <= 0:
            raise ReadinessTimeout("HARNESS_INVALID initial target I/O deadline")
        self.s.settimeout(min(remaining, quantum))

    def _send(self, data):
        if self._budget is None:
            self.s.sendall(data)
            return
        offset = 0
        while offset < len(data):
            self._service_io()
            try:
                sent = self.s.send(data[offset:])
            except socket.timeout:
                continue
            if not sent:
                raise ConnectionError("S4 plugin closed while sending")
            offset += sent
        self._service_io()

    def _read(self, n):
        buf = bytearray()
        while len(buf) < n:
            self._service_io()
            try:
                chunk = self.s.recv(n - len(buf))
            except socket.timeout:
                if self._budget is None:
                    raise
                continue
            if not chunk:
                raise ConnectionError("S4 plugin closed the connection")
            buf += chunk
        self._service_io()
        return bytes(buf)

    def set_target(self, x, y, z):
        self._send(b"T" + struct.pack("<3d", x, y, z))
        return struct.unpack("<d", self._read(8))[0]

    def frames(self):
        self._send(b"R")
        magic, version, now, lt, rt, dt, at, count, lh, lw, rw, rh, dw, dh = HDR.unpack(self._read(HDR.size))
        if magic != b"MS4F" or version != 1:
            self.close()
            raise ValueError("bad S4 frame header %r %r" % (magic, version))
        # The v1 rig is fixed. Validate BEFORE allocation/read of peer-sized payloads.
        # A not-yet-produced modality has both dimensions zero, never mixed zeros.
        if ((lh, lw) not in ((0, 0), (H, W)) or (rw, rh) not in ((0, 0), (CAM_W, CAM_H))
                or (dw, dh) not in ((0, 0), (CAM_W, CAM_H))):
            self.close()
            raise ValueError("bad S4 sensor dimensions")
        ranges = np.frombuffer(self._read(8 * lh * lw), "<f8").reshape(lh, lw) if lh * lw else None
        rgb = np.frombuffer(self._read(rw * rh * 3), np.uint8).reshape(rh, rw, 3) if rw * rh else None
        depth = np.frombuffer(self._read(4 * dw * dh), "<f4").reshape(dh, dw) if dw * dh else None
        return {"sim_now": now, "lidar_t": lt, "rgb_t": rt, "depth_t": dt, "target_t": at, "target_count": count,
                "ranges": ranges, "rgb": rgb, "depth": depth}

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


def wait_initial_target(client, send_target, *, timeout_s=2.0, poll_s=0.01,
                        maintain_hold=lambda: None, clock=time.monotonic, sleep=time.sleep,
                        sensor_mode='rgbd_lidar'):
    """First apply + strictly later required sensors; no capture-bound ID in v1.

    Requires exclusive ownership of a fresh bridge. No policy/tracker call occurs
    during polling. Receipt ACK is not application ACK. This only repairs startup.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0 or not 0 < poll_s <= 0.025:
        raise ValueError("positive finite timeout and poll <= 0.025 s required")
    if sensor_mode not in ('rgbd', 'rgbd_lidar'):
        raise ValueError('sensor_mode must be rgbd or rgbd_lidar')
    stamps = ('rgb_t', 'depth_t') + (('lidar_t',) if sensor_mode == 'rgbd_lidar' else ())
    payloads = ('rgb', 'depth') + (('ranges',) if sensor_mode == 'rgbd_lidar' else ())
    started = clock()
    deadline = started + timeout_s

    def header(frame):
        try:
            count = frame['target_count']
            ts = {k: float(frame[k]) for k in ('sim_now', 'target_t') + stamps}
            if type(count) is not int or count < 0 or not all(math.isfinite(v) for v in ts.values()):
                raise ValueError()
            return count, ts
        except (KeyError, TypeError, ValueError) as exc:
            raise ReadinessError('HARNESS_INVALID initial target header') from exc

    with client.bounded_io(deadline, maintain_hold, clock=clock):
        count, base = header(client.frames())
        if count != 0 or base['target_t'] != -1:
            raise ReadinessError('HARNESS_INVALID initial target requires fresh bridge')
        receipt = float(send_target())
        if not math.isfinite(receipt) or receipt < base['sim_now']:
            raise ReadinessError('HARNESS_INVALID target receipt clock')
        previous_count, previous_ts, applied, polls = 0, base, None, 0
        while True:
            frame = client.frames()
            polls += 1
            count, ts = header(frame)
            if count > 1 or count < previous_count or any(
                    ts[k] < previous_ts[k] for k in ('sim_now',) + stamps):
                raise ReadinessError('HARNESS_INVALID target count/simulation regression')
            previous_count, previous_ts = count, ts
            if count == 0 and ts['target_t'] != -1:
                raise ReadinessError('HARNESS_INVALID unapplied target metadata')
            if count == 1:
                if ts['target_t'] < base['sim_now'] or (applied is not None and applied != ts['target_t']):
                    raise ReadinessError('HARNESS_INVALID target application clock')
                applied = ts['target_t']
                # Apply may precede the server's receipt reply under thread scheduling.
                fresh = (ts['sim_now'] >= max(receipt, applied)
                         and all(applied < ts[k] <= ts['sim_now'] for k in stamps))
                if fresh and all(getattr(frame.get(k), 'size', 0) > 0 for k in payloads):
                    report = {'scope': 'initial apply and strictly later nonempty sensor buffers',
                              'sensor_mode': sensor_mode, 'required_payloads': list(payloads),
                              'receipt_sim_s': receipt, 'applied_sim_s': applied, 'target_count': count,
                              'polls': polls, 'preflight_packets': polls + 1, 'wall_s': clock() - started,
                              'timeout_s': timeout_s, 'io_service_quantum_s': 0.025,
                              'capture_bound_target_identity': 'NOT_RECORDED: MS4F v1 limitation'}
                    getattr(client, 'record_initial_target_readiness', lambda r: None)(report)
                    return frame, report
            maintain_hold()
            remaining = deadline - clock()
            if remaining <= 0:
                raise ReadinessTimeout('HARNESS_INVALID initial target/frame timeout')
            sleep(min(poll_s, remaining))


def motar_lidar(ranges_gz):
    """Gazebo gpu_ray ranges (vertical -10..+20 deg rows, horizontal -175..180 deg columns) -> MOTAR's (1, 288)
    metres: row i = elevation 20 - 10 i, column j = azimuth 180 - 5 j; beyond 12 m or no return -> 12; below 0.2 m
    -> 0 (MOTAR's -1 marker after its clamp to [0, 1])."""
    r = np.asarray(ranges_gz, np.float64)
    if r.shape != (H, W):
        raise ValueError("expected Gazebo ranges (4, 72), got %s" % (r.shape,))
    m = r[::-1, ::-1].copy()
    too_close = np.isneginf(m) | (np.isfinite(m) & (m < LIDAR_MIN))
    m[np.isnan(m) | np.isposinf(m) | (m > LIDAR_MAX)] = LIDAR_MAX
    m[too_close] = 0.0
    return m.astype(np.float32).reshape(1, H * W)


def ray_norms(w=CAM_W, h=CAM_H, hfov=CAM_HFOV):
    """|ray| for a unit-depth pinhole pixel ray through each pixel centre (square pixels, Gazebo's camera)."""
    f = (w / 2.0) / math.tan(hfov / 2.0)
    u = (np.arange(w) + 0.5 - w / 2.0) / f
    v = (np.arange(h) + 0.5 - h / 2.0) / f
    return np.sqrt(1.0 + u[None, :] ** 2 + v[:, None] ** 2)


_NORMS = ray_norms()


def motar_rgbd(rgb_u8, depth_z):
    """Gazebo RGB (h, w, 3) uint8 and depth along the optical axis (h, w) -> MOTAR rgb (1, 3, h, w) float32 in [0, 1]
    and depth (1, h, w) float32 along each pixel's ray, no return or beyond 20 m -> 20 m."""
    rgb = np.asarray(rgb_u8, np.float32).transpose(2, 0, 1)[None] / 255.0
    z = np.asarray(depth_z, np.float64)
    rng = z * _NORMS
    rng[~np.isfinite(rng) | (rng > CAM_FAR) | (z <= 0)] = CAM_FAR
    return rgb.astype(np.float32), rng.astype(np.float32)[None]


def to_gazebo_xy(p_world_xy, p0, yaw0, spawn, rot):
    gx, gy = rot(-yaw0, p_world_xy[0] - p0[0], p_world_xy[1] - p0[1])
    return gx + spawn[0], gy + spawn[1]


def world_text_s4(s3b_world_text, *, sensor_mode='rgbd_lidar', model='navrl_quad_s4'):
    """The S3b world plus the target and the plugin."""
    target = ("<model name='%s'><pose>0 0 -10 0 0 0</pose><link name='l'><gravity>0</gravity><kinematic>1</kinematic>"
              "<visual name='v'><geometry><sphere><radius>%.3f</radius></sphere></geometry><material>"
              "<ambient>%.3f %.3f %.3f 1</ambient><diffuse>%.3f %.3f %.3f 1</diffuse><specular>0 0 0 1</specular>"
              "</material></visual></link></model>" % ((TARGET_MODEL, TARGET_RADIUS) + TARGET_RGB + TARGET_RGB))
    if sensor_mode not in ('rgbd', 'rgbd_lidar') or model not in ('navrl_quad_s4', 'navrl_quad_rgbd'):
        raise ValueError('unknown S4 sensor mode/model')
    scope = 'default::%s::base_link::' % model
    lidar = '<lidar>%smotar_lidar</lidar>' % scope if sensor_mode == 'rgbd_lidar' else ''
    library = 'libmotar_rgbd_bridge.so' if sensor_mode == 'rgbd' else 'libmotar_s4_bridge.so'
    plugin = ("<plugin name='motar_s4' filename='%s'><port>%d</port><target>%s</target>"
              "%s<rgb>%smotar_rgb</rgb><depth>%smotar_depth</depth></plugin>"
              % (library, PORT, TARGET_MODEL, lidar, scope, scope))
    if s3b_world_text.count("</world>") != 1:
        raise ValueError("expected one </world>")
    return s3b_world_text.replace("</world>", target + "\n" + plugin + "\n</world>", 1)


def gazebo_episode_class(B):
    class GazeboSensorEpisode(B.Episode):
        """tools/sitl_bridge.Episode.observe with the sensor arrays supplied (Gazebo's), plus a shadow of MOTAR's own
        render at the same pose for the record. Perception, policy input and the governor see only Gazebo's arrays."""

        def observe_with(self, s, rgb, depth, lidar):
            torch = self.torch
            T = lambda x: torch.as_tensor(x, dtype=torch.float32).reshape(1, -1)          # noqa: E731
            rpos = s.get("render_pos", s["pos"])
            rvq = s.get("render_vquat", s["vquat"])
            rfq = s.get("render_fquat", s["fquat"])
            m_rgb, m_depth = self.detector.render_raw_rgbd(T(rpos), T(rvq), T(s["tpos"]), None)
            m_lidar = self.lidar_m(rpos, rfq, s["tpos"])
            far = 10.0                     # MOTAR fills no-return obstacle pixels with 10 m, Gazebo's here with 20 m
            shadow = {"motar_target_px": int((self.detector.target_mask > 0).sum()),
                      "lidar_abs_diff_median_m": float((m_lidar - torch.as_tensor(lidar)).abs().median()),
                      "lidar_abs_diff_max_m": float((m_lidar - torch.as_tensor(lidar)).abs().max()),
                      "depth_abs_diff_median_m": float((m_depth.clamp(max=far)
                                                        - torch.as_tensor(depth).clamp(max=far)).abs().median())}
            g_lidar = torch.as_tensor(lidar, dtype=torch.float32).reshape(1, -1)
            obs, diag = self.perception.observe(
                rgb=torch.as_tensor(rgb, dtype=torch.float32), depth=torch.as_tensor(depth, dtype=torch.float32),
                lidar_m=g_lidar, drone_pos_w=T(s["pos"]), drone_vel_w=T(s["vel"]), vehicle_quat=T(s["vquat"]),
                yaw_rate=T(s["angvel"])[:, 2] / float(self.tc.yaw_rate_max), previous_action=T(s["prev_action"]),
                max_velocity=self.tc.max_velocity, flight_altitude=self.tc.flight_altitude, training=False,
                env_bounds_min=T(s["bounds_min"]), env_bounds_max=T(s["bounds_max"]))
            return obs, diag, g_lidar, shadow

    return GazeboSensorEpisode


# ------------------------------------------------------------------------------------- ground removal (S4-ground arm)
GROUND_Z_M = 0.10                     # a return whose hit point lies below this height above the ground is ground
CAM_UP_M = 0.03                       # camera mount height above the body origin (the LiDAR is at the origin)


def frd_to_ned_rotate(q_wxyz, v):
    """Rotate body-FRD vectors v (..., 3) by PX4's attitude quaternion (w, x, y, z), body FRD -> NED."""
    w, x, y, z = q_wxyz
    u = np.array([x, y, z], np.float64)
    v = np.asarray(v, np.float64)
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def _flu_dirs_lidar():
    el = np.radians(20.0 - 10.0 * np.arange(H))                       # MOTAR rows
    az = np.radians(180.0 - 5.0 * np.arange(W))                       # MOTAR columns
    E, A = np.meshgrid(el, az, indexing="ij")
    return np.stack([np.cos(A) * np.cos(E), np.sin(A) * np.cos(E), np.sin(E)], -1)


def _flu_dirs_camera(w=CAM_W, h=CAM_H, hfov=CAM_HFOV):
    f = (w / 2.0) / math.tan(hfov / 2.0)
    u = (np.arange(w) + 0.5 - w / 2.0) / f
    v = (np.arange(h) + 0.5 - h / 2.0) / f
    U, V = np.meshgrid(u, v)
    d = np.stack([np.ones_like(U), -U, -V], -1)
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


_LIDAR_FLU, _CAM_FLU = _flu_dirs_lidar(), _flu_dirs_camera()


def up_components(q_wxyz, dirs_flu):
    frd = dirs_flu * np.array([1.0, -1.0, -1.0])
    return -frd_to_ned_rotate(q_wxyz, frd)[..., 2]


def remove_ground(lidar_m, depth, q_wxyz, height_m):
    """S4-ground: Gazebo returns that hit the ground plane become no-return, using the vehicle's true attitude and
    height above the ground (as an onboard system would use its estimator's). LiDAR (1, 288) m: a ground hit -> 12 m;
    depth (1, 90, 160) m: a ground pixel -> 20 m. Returns copies and the counts removed."""
    lm = np.array(lidar_m, np.float64).reshape(H, W)
    up = up_components(q_wxyz, _LIDAR_FLU)
    hit_z = height_m + lm * up
    g_l = (lm > 0) & (lm < LIDAR_MAX) & (hit_z < GROUND_Z_M)
    lm[g_l] = LIDAR_MAX
    dm = np.array(depth, np.float64).reshape(CAM_H, CAM_W)
    upc = up_components(q_wxyz, _CAM_FLU)
    hz = height_m + CAM_UP_M + dm * upc
    g_d = (dm < CAM_FAR) & (hz < GROUND_Z_M)
    dm[g_d] = CAM_FAR
    return (lm.astype(np.float32).reshape(1, H * W), dm.astype(np.float32)[None],
            {"lidar_ground_removed": int(g_l.sum()), "depth_ground_removed": int(g_d.sum())})
