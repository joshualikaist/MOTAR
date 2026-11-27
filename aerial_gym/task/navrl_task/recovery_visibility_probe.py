"""Evaluation-only centre LOS from the detector camera and its actual scene mesh.

Importing this module, or constructing an OFF factory, imports no torch/Warp/simulator.
The supported ON path lazily queries the same scene mesh as the target renderer, using
the camera pose already written by that render. It never rerenders or edits detector data.
The caller attests render identity with matching tokens; these are not sensor timestamps.
Centre-ray blockage is not full silhouette occlusion. GPU runtime parity is NOT_TESTED.
"""
import copy
import importlib
import math

CONTRACT = "MOTAR_ACTUAL_SCENE_MESH_CENTER_LOS_V1"
UNKNOWN_CONTRACT = "MOTAR_RECOVERY_GEOMETRY_UNKNOWN_V1"
NOT_RECORDED = "NOT_RECORDED"


def _vector(values, size):
    out = tuple(float(value) for value in values)
    if len(out) != size or not all(math.isfinite(value) for value in out):
        raise ValueError("geometry vector has invalid dimensions or non-finite values")
    return out


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def camera_center_geometry(origin, orientation_xyzw, target, hfov_rad, vfov_rad, max_range_m):
    """Independent scalar reference: centre frustum and Euclidean centre range, separately.

    x forward / y left / z up; quaternion is the actual rendered camera's world rotation.
    The finite target surface can still render when its centre is outside either condition.
    """
    eye, point, quat = _vector(origin, 3), _vector(target, 3), _vector(orientation_xyzw, 4)
    if abs(_dot(quat, quat) - 1.0) > 1e-4:
        raise ValueError("camera quaternion is not unit length")
    if not (0. < hfov_rad < math.pi and 0. < vfov_rad < math.pi
            and math.isfinite(max_range_m) and max_range_m > 0.):
        raise ValueError("unsupported camera frustum")
    delta = tuple(t - e for t, e in zip(point, eye))
    # Inverse xyzw quaternion rotation; no alteration of the caller's quaternion.
    q = tuple(-v for v in quat[:3])
    cross = _cross(q, delta)
    second = _cross(q, cross)
    local = tuple(delta[i] + 2. * quat[3] * cross[i] + 2. * second[i] for i in range(3))
    distance = math.sqrt(_dot(delta, delta))
    inside = (local[0] > 0. and abs(local[1]) <= local[0] * math.tan(hfov_rad / 2.)
              and abs(local[2]) <= local[0] * math.tan(vfov_rad / 2.))
    return {"target_center_in_fov": inside,
            "target_center_in_range": 0. < distance <= max_range_m,
            "center_range_m": distance}


def segment_triangle_intersection(start, end, a, b, c, epsilon=1e-10):
    """Double-precision reference segment parameter, or None; edge hits count.

    Coplanar rays are not classified as crossings by this ray-triangle reference.
    This analytic fixture is not a replacement for the actual scene-mesh query.
    """
    start, end, a, b, c = (_vector(v, 3) for v in (start, end, a, b, c))
    direction = tuple(y - x for x, y in zip(start, end))
    e1, e2 = tuple(y - x for x, y in zip(a, b)), tuple(y - x for x, y in zip(a, c))
    h = _cross(direction, e2)
    determinant = _dot(e1, h)
    if abs(determinant) <= epsilon:
        return None
    rel = tuple(x - y for x, y in zip(start, a))
    u = _dot(rel, h) / determinant
    q = _cross(rel, e1)
    v = _dot(direction, q) / determinant
    fraction = _dot(e2, q) / determinant
    if u < -epsilon or v < -epsilon or u + v > 1. + epsilon or not -epsilon <= fraction <= 1. + epsilon:
        return None
    return max(0., min(1., fraction))


def reference_mesh_blocked(start, end, vertices, triangle_indices):
    """Analytic triangle fixture, using explicit vertices rather than scene GT boxes."""
    if len(triangle_indices) % 3:
        raise ValueError("triangle index count is not divisible by three")
    return any(segment_triangle_intersection(start, end, *(vertices[i] for i in triangle_indices[j:j + 3]))
               is not None for j in range(0, len(triangle_indices), 3))


def reference_aabb_blocked(start, end, lower, upper):
    """Closed slab fixture for tests only; never used as the live actual-mesh oracle."""
    start, end, lower, upper = (_vector(v, 3) for v in (start, end, lower, upper))
    lo, hi = 0., 1.
    for a, b, minimum, maximum in zip(start, end, lower, upper):
        if minimum > maximum:
            raise ValueError("invalid AABB")
        direction = b - a
        if abs(direction) < 1e-12:
            if not minimum <= a <= maximum:
                return False
        else:
            near, far = sorted(((minimum - a) / direction, (maximum - a) / direction))
            lo, hi = max(lo, near), min(hi, far)
            if lo > hi:
                return False
    return True


def _center_los_kernel(mesh_ids, origins, targets, blockage):
    # wp is bound, and these annotations are installed, only by the explicit lazy ON path.
    index = wp.tid()
    ray = targets[index] - origins[index]
    length = wp.length(ray)
    blockage[index] = wp.int32(-1)
    if length > 1.0e-8:
        direction = ray / length
        distance = float(0.0)
        u = float(0.0)
        v = float(0.0)
        sign = float(0.0)
        normal = wp.vec3()
        face = int(0)
        blocked = wp.mesh_query_ray(mesh_ids[index], origins[index], direction, length,
                                    distance, u, v, sign, normal, face)
        blockage[index] = wp.int32(0)
        if blocked:
            blockage[index] = wp.int32(1)


_WARP_KERNEL = None
wp = None  # Bound only on the explicit lazy ON path; no module import or initialization here.


def _load_warp_kernel():
    global _WARP_KERNEL, wp
    if _WARP_KERNEL is None:
        wp = importlib.import_module("warp")
        wp.init()
        _center_los_kernel.__annotations__ = {
            "mesh_ids": wp.array(dtype=wp.uint64), "origins": wp.array(dtype=wp.vec3),
            "targets": wp.array(dtype=wp.vec3), "blockage": wp.array(dtype=wp.int32)}
        _WARP_KERNEL = wp.kernel(_center_los_kernel)
    return wp, _WARP_KERNEL


def _host_values(value):
    if hasattr(value, "detach"):
        return value.detach().cpu().tolist()
    if hasattr(value, "tolist"):
        return value.tolist()
    return copy.deepcopy(value)


def rendered_frustum(detector):
    """Use the actual render FOV scale, not perception's nominal camera intrinsics."""
    scale = 1. + float(getattr(detector, "fov_scale_err", 0.))
    return float(detector.hfov) * scale, float(detector.vfov) * scale, float(detector.max_range)


class RecoveryVisibilityProbe:
    """Observer-owned mesh probe; no tensor is written through a detector reference."""

    def __init__(self, detector, *, target_dynamics, scene_contract):
        self.detector = detector
        self.target_dynamics = target_dynamics
        self.scene_contract = scene_contract
        self._last_token = None
        self._last_rows = None
        self._invalidated_token = None
        self._owned_buffers = None

    def invalidate(self):
        """Call at reset; no old frame may be reused as a fresh render."""
        if self._last_token is not None:
            self._invalidated_token = copy.deepcopy(self._last_token)
        self._last_token = None
        self._last_rows = None

    def _unsupported(self):
        d = self.detector
        if self.target_dynamics not in ("legacy", "bounded") or getattr(d, "target_use_oriented_box", True):
            return "UNSUPPORTED_PHYSICAL_OR_ORIENTED_TARGET"
        if self.scene_contract != "static_scene_mesh":
            return "UNSUPPORTED_SCENE_CONTRACT"
        if getattr(d, "detect_decoupled", True):
            return "UNSUPPORTED_DECOUPLED_CAMERA"
        if (getattr(d, "target_render_mode", None) != "analytic_flat"
                or getattr(d, "_dynamic_mesh_treatment", None) is not None):
            return "UNSUPPORTED_DYNAMIC_TARGET_GEOMETRY"
        for name in ("mesh_ids", "_origins", "_orientations", "_targets", "target_mask", "hfov", "vfov", "max_range"):
            if not hasattr(d, name):
                return "MISSING_RENDER_INPUT_" + name
        return None

    def _unknown(self, token, reason):
        return [{"target_center_in_fov": None, "target_center_in_range": None,
                 "center_los_blocked": None, "geometry_contract": UNKNOWN_CONTRACT,
                 "rendered_target_visible_pixels": None, "full_visual_occlusion": NOT_RECORDED,
                 "status": "UNKNOWN", "reason": reason, "capture_token": copy.deepcopy(token),
                 "sensor_capture_time_s": NOT_RECORDED}
                for _ in range(int(self.detector.num_envs))]

    def snapshot(self, capture_token, *, render_token):
        """Read a caller-attested fresh render; missing/stale tokens produce UNKNOWN.

        The detector has no native exposure/frame stamp. Matching caller tokens attest the
        adapter invoked this after its render; they cannot establish a hardware capture time.
        Repeated requests for that token return copied cached scalars without another query.
        """
        if capture_token is None or render_token is None or capture_token != render_token:
            return self._unknown(capture_token, "MISSING_OR_MISMATCHED_RENDER_TOKEN")
        if capture_token == self._invalidated_token:
            return self._unknown(capture_token, "INVALIDATED_RENDER_TOKEN")
        if capture_token == self._last_token and self._last_rows is not None:
            return copy.deepcopy(self._last_rows)
        unsupported = self._unsupported()
        if unsupported:
            return self._unknown(capture_token, unsupported)
        d = self.detector
        origins, quats, targets = (_host_values(getattr(d, name)) for name in ("_origins", "_orientations", "_targets"))
        mask = _host_values(d.target_mask)
        n = int(d.num_envs)
        if len(origins) != n or len(quats) != n or len(targets) != n or len(mask) != n:
            return self._unknown(capture_token, "RENDER_BATCH_SHAPE_MISMATCH")
        try:
            hfov, vfov, far_plane = rendered_frustum(d)
            rows = [camera_center_geometry(eye, quat, target, hfov, vfov, far_plane)
                    for eye, quat, target in zip(origins, quats, targets)]
        except (ValueError, TypeError):
            return self._unknown(capture_token, "INVALID_RENDER_CAMERA_GEOMETRY")
        wp_module, kernel = _load_warp_kernel()
        device = d.mesh_ids.device
        if tuple(d.mesh_ids.shape) != (n,):
            return self._unknown(capture_token, "SCENE_MESH_BATCH_SHAPE_MISMATCH")
        # These inputs and outputs are independently allocated; existing sensor buffers are read only.
        eye_buffer = wp_module.array(origins, dtype=wp_module.vec3, device=device)
        target_buffer = wp_module.array(targets, dtype=wp_module.vec3, device=device)
        output_buffer = wp_module.zeros(n, dtype=wp_module.int32, device=device)
        self._owned_buffers = (eye_buffer, target_buffer, output_buffer)
        wp_module.launch(kernel, dim=n, inputs=[d.mesh_ids, eye_buffer, target_buffer],
                         outputs=[output_buffer], device=device)
        blockages = output_buffer.numpy().tolist()
        for index, row in enumerate(rows):
            row.update(center_los_blocked=(None if blockages[index] < 0 else bool(blockages[index])),
                       geometry_contract=CONTRACT, rendered_target_visible_pixels=sum(
                           int(value > 0) for line in mask[index] for value in line),
                       full_visual_occlusion=NOT_RECORDED, capture_token=copy.deepcopy(capture_token),
                       camera_origin_world=origins[index], camera_orientation_xyzw=quats[index],
                       target_center_world=targets[index], status="PROBED",
                       reason=None, sensor_capture_time_s=NOT_RECORDED,
                       actual_hfov_rad=hfov, actual_vfov_rad=vfov, camera_far_plane_m=far_plane,
                       target_geometry="ANALYTIC_VIRTUAL_SPHERE", target_dynamics=self.target_dynamics,
                       scene_contract=self.scene_contract,
                       render_identity_provenance="CALLER_ATTESTED_SAME_RENDER",
                       runtime_parity="NOT_TESTED_SEPARATE_GPU_APPROVAL_REQUIRED")
        self._last_token, self._last_rows = copy.deepcopy(capture_token), copy.deepcopy(rows)
        return rows


def build_visibility_probe(detector, *, enabled=False, target_dynamics="bounded", scene_contract="static_scene_mesh"):
    """Default OFF performs no construction, imports, buffers, mesh queries or initialization."""
    if not enabled:
        return None
    return RecoveryVisibilityProbe(detector, target_dynamics=target_dynamics, scene_contract=scene_contract)
