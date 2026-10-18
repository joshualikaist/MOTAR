"""P02 evaluator-only spatial association of delivered detector support with target pixels.

The actual support is output[5] of perception._detect_rgbd, never the renderer's bbox.
GT is read only inside this recorder and exported as scoring telemetry; the original detector
outputs are returned verbatim. This implements docs/plans/h_r3_p02.json, not persistent identity.
"""
from __future__ import annotations

import torch

CENTROID_EPS = 1e-5


def score_detection(mask, target_support, measurement_vehicle, delivered, camera_offset,
                    fx, fy, cx, cy):
    """Pure batched pixel scoring; empty bboxes are [-1,-1,-1,-1], empty ratios are zero.

    Centroid proximity is continuous Chebyshev distance to a visible pixel CENTER. Rounding
    followed by a raster dilation would implement a different (up to 1.5-pixel) tolerance.
    """
    if mask.ndim != 3 or target_support.shape != mask.shape:
        raise ValueError("identity scoring requires equal [N,H,W] detector and target support")
    n, h, w = mask.shape
    if n < 1 or h < 1 or w < 1 or measurement_vehicle.shape != (n, 3) or delivered.shape != (n,):
        raise ValueError("invalid identity batch, measurement or camera flag shape")
    if mask.device != target_support.device or mask.device != measurement_vehicle.device or mask.device != delivered.device:
        raise ValueError("identity inputs must share a device")
    if not all(float(x) == float(x) and abs(float(x)) < float("inf") for x in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError("invalid detector intrinsics")
    detected = mask.detach().bool()
    target = target_support.detach().bool()
    camera = delivered.detach().bool()
    dp = detected.sum(dim=(1, 2))
    vp = target.sum(dim=(1, 2))
    overlap = (detected & target).sum(dim=(1, 2))
    dominance = overlap.float() / dp.clamp(min=1)
    recall = overlap.float() / vp.clamp(min=1)
    iou = overlap.float() / (dp + vp - overlap).clamp(min=1)

    offset = torch.as_tensor(camera_offset, device=mask.device, dtype=measurement_vehicle.dtype)
    if offset.shape not in ((3,), (1, 3)):
        raise ValueError("camera offset must be shared [3] or [1,3]")
    c = measurement_vehicle.detach() - offset
    forward = torch.isfinite(c).all(dim=1) & (c[:, 0] > 0)
    safe_x = torch.where(forward, c[:, 0], torch.ones_like(c[:, 0]))
    uv = torch.stack((cx - fx * c[:, 1] / safe_x, cy - fy * c[:, 2] / safe_x), dim=1)
    finite_uv = torch.isfinite(uv).all(dim=1)
    valid_centroid = forward & finite_uv
    # At most nine pixel centers can lie within this radius. Enumerating around the nearest
    # center avoids an N*H*W floating distance tensor and still checks the continuous distance.
    safe_uv = torch.where(torch.isfinite(uv), uv, torch.zeros_like(uv))
    center = safe_uv.clamp(min=-2, max=max(h, w) + 2).round().long()
    near = torch.zeros(n, dtype=torch.bool, device=mask.device)
    batch = torch.arange(n, device=mask.device)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            col, row = center[:, 0] + dx, center[:, 1] + dy
            inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
            close = ((safe_uv[:, 0] - col).abs() <= 1.0 + CENTROID_EPS) & \
                    ((safe_uv[:, 1] - row).abs() <= 1.0 + CENTROID_EPS)
            near |= inside & close & target[batch, row.clamp(0, h - 1), col.clamp(0, w - 1)]
    near &= valid_centroid

    cols = torch.arange(w, device=mask.device).view(1, 1, w)
    rows = torch.arange(h, device=mask.device).view(1, h, 1)
    bbox = torch.stack((torch.where(detected, cols, w).amin(dim=(1, 2)),
                        torch.where(detected, rows, h).amin(dim=(1, 2)),
                        torch.where(detected, cols, -1).amax(dim=(1, 2)),
                        torch.where(detected, rows, -1).amax(dim=(1, 2))), dim=1)
    bbox = torch.where((dp > 0).unsqueeze(1), bbox, torch.full_like(bbox, -1))
    eligible = camera & (overlap >= 1) & near
    return {"detector_px": dp, "visible_px": vp, "overlap_px": overlap,
            "detector_false_px": dp - overlap, "detector_overlap_fraction": dominance,
            "target_recall": recall, "mask_iou": iou, "detector_bbox": bbox,
            "reported_centroid_uv": uv, "centroid_near_visible": near,
            "camera_delivered": camera.clone(),
            "camera_identity_correct": eligible & (dominance >= 0.5),
            "camera_identity_correct_025": eligible & (dominance >= 0.25),
            "camera_identity_correct_075": eligible & (dominance >= 0.75)}


def _guard(perception, detector, training=False):
    """Unsupported measurement/support relationships are integrity failures, never proxies."""
    if (perception.detect_decoupled or getattr(detector, "detect_decoupled", False)
            or perception.detect_width != perception.width or perception.detect_height != perception.height
            or detector.width != perception.width or detector.height != perception.height):
        raise ValueError("P02 identity requires equal camera/detect resolution")
    if perception.detection_latency_s != 0 or perception._latency_steps != 0:
        raise ValueError("P02 identity requires zero detection latency")
    if perception.empirical_error is not None:
        raise ValueError("P02 identity does not support empirical measurement errors")
    if (perception._detector_noise_active or perception.range_error_m != 0
            or getattr(perception, "_pose_premise_active", False)
            or getattr(perception, "target_mask_backfill", False)
            or getattr(perception, "latency_obstacle_fix", "off") != "off"):
        raise ValueError("P02 identity does not support measurement perturbations/backfill")
    if training and any(getattr(perception, key, 0) != 0 for key in ("rgb_noise_std", "depth_noise_std", "dropout_prob")):
        raise ValueError("P02 identity does not support active training perturbations")


class CameraIdentityRecorder:
    """Call-through instance wrapper; latest is replaced per detector invocation, not accumulated."""

    def __init__(self, perception, detector):
        _guard(perception, detector)
        if getattr(perception._detect_rgbd, "_p02_identity_wrapper", False):
            raise ValueError("P02 camera identity recorder already attached")
        self.perception, self.detector = perception, detector
        self.latest, self.call_count, self._closed = None, 0, False
        self._had_instance_method = "_detect_rgbd" in vars(perception)
        self._original = perception._detect_rgbd

        def detect(*args, **kwargs):
            self.latest = None   # any failed invocation must invalidate the previous frame
            training = kwargs.get("training", args[2] if len(args) > 2 else False)
            _guard(perception, detector, training)
            output = self._original(*args, **kwargs)
            measurement, delivered, mask = (output[i].detach().clone() for i in (0, 3, 5))
            visible = (detector.target_mask.detach() > 0).clone()
            with torch.no_grad():
                self.latest = score_detection(mask, visible, measurement, delivered,
                                              perception.camera_offset, perception.detect_fx,
                                              perception.detect_fy, perception.detect_cx, perception.detect_cy)
            self.call_count += 1
            return output

        detect._p02_identity_wrapper = True
        self._wrapper = detect
        perception._detect_rgbd = detect

    def close(self):
        """Restore the original method without overwriting an intervening wrapper."""
        if self._closed:
            return
        if self.perception._detect_rgbd is not self._wrapper:
            raise RuntimeError("P02 identity detector wrapper was replaced externally")
        if self._had_instance_method:
            self.perception._detect_rgbd = self._original
        else:
            del self.perception._detect_rgbd
        self._closed = True


def attach_camera_identity(perception, detector):
    return CameraIdentityRecorder(perception, detector)
