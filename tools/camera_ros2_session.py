"""ROS-message buffering and publication boundary for the existing camera policy.

No rclpy, device, graph, flight mode, or publisher is constructed here. Callbacks
and prepare/finish run on the caller's serialized executor. Only infer runs in a
single worker. Both capture-clock age and monotonic receipt age are checked.
Rejected post-inference work is discarded and the policy requires explicit reset.

Altitude gating: while the estimator height puts the camera outside the runtime's floor/ceiling
filter band (runtime.height_band, e.g. on the ground before take-off), the worker does not call
runtime.step, finish() returns no proposal and the session reports WAITING_ALTITUDE without a
fault and without advancing the runtime history. Once the first gated tick has finished, input
freshness, clock and 10 Hz period checks apply exactly as for a running session. The first in-band
frame after gated ticks starts from a fresh runtime history (runtime.reset(), as a reset does).
"""
import copy
from dataclasses import dataclass
import itertools
import math
from collections import deque

import numpy as np

from camera_ros2_adapter import MAX_POSE_BRACKET_S, build_frame, stamp_seconds, validate_binding

KINDS = ('rgb', 'depth', 'camera_info', 'odometry')
MAX_AGE_S, MAX_SKEW_S, CONTROL_DT_S = .15, .025, .1


class SessionFault(RuntimeError):
    pass


@dataclass(frozen=True)
class WorkItem:
    epoch: int
    sequence: int
    messages: dict
    stamps: tuple
    receipts: tuple
    ros_now_s: float
    monotonic_now_s: float
    # (before, after) odometry messages bracketing the RGB stamp, or None (nearest only).
    odometry_bracket: tuple = None
    # True after altitude-gated ticks: an in-band frame resets the runtime history before step.
    fresh_history: bool = False


class RosPolicySession:
    """One sequential, explicitly resettable ROS policy session."""
    def __init__(self, runtime, binding):
        self.binding = copy.deepcopy(validate_binding(binding))
        self.runtime = runtime
        self.epoch = 0
        self._busy = False
        self.reset()

    def reset(self):
        if self._busy:
            raise SessionFault('inference still running; wait for completion before reset')
        self.runtime.reset()
        self.epoch += 1
        self.sequence = 0
        self.fault = None
        self.started = False
        # live: at least one tick finished (a proposal or an altitude-gated tick); freshness
        # watchdogs apply from then on. started: at least one proposal.
        self.live = False
        self.waiting_altitude = False
        self.altitude = None
        self._fresh_history = False
        self._queues = {k: deque(maxlen=8) for k in KINDS}
        self._last_seen = {}
        self._last_used = None
        self._last_start_ros = None
        self._last_success_wall = None
        self._clock_ros = None
        self._clock_advanced_wall = None
        self._calibration = None
        self._pending = None
        self.dropped_old_messages = 0

    @property
    def status(self):
        if self.fault:
            return 'FAULT'
        if self.waiting_altitude:
            return 'WAITING_ALTITUDE'
        return 'RUNNING_PROPOSALS' if self.started else 'WAITING_FOR_INPUTS'

    def invalidate(self, reason):
        if self.fault is None:
            self.fault = str(reason)
            self.epoch += 1
        # The worker may currently be mutating Torch history. Invalidate it only
        # after that worker returns; the separate session latch rejects output now.
        if not self._busy:
            self.runtime.invalidate()

    def _raise(self, reason):
        self.invalidate(reason)
        raise SessionFault(str(reason))

    def offer(self, kind, message, *, received_monotonic_s):
        if kind not in KINDS:
            raise ValueError('unknown ROS input kind')
        if self.fault:
            return False
        received = float(received_monotonic_s)
        if not math.isfinite(received) or received < 0:
            self._raise('invalid monotonic input receipt')
        try:
            stamp = stamp_seconds(message.header.stamp)
        except (ValueError, TypeError, AttributeError) as exc:
            self._raise('malformed %s capture stamp: %s' % (kind, exc))
        if kind in self._last_seen and stamp <= self._last_seen[kind]:
            self.dropped_old_messages += 1
            return False  # Never renew receipt freshness for replayed samples.
        if kind == 'camera_info':
            roi = message.roi
            calibration = (message.header.frame_id, message.width, message.height,
                           tuple(message.k), tuple(message.p), tuple(message.r), tuple(message.d),
                           getattr(message, 'distortion_model', None),
                           message.binning_x, message.binning_y,
                           roi.x_offset, roi.y_offset, roi.width, roi.height, roi.do_rectify)
            if self._calibration is not None and calibration != self._calibration:
                self._raise('camera calibration changed; explicit reset/rebinding required')
            self._calibration = calibration
        self._queues[kind].append((stamp, received, copy.deepcopy(message)))
        self._last_seen[kind] = stamp
        return True

    def _clocks(self, now_ros_s, now_monotonic_s):
        now, wall = float(now_ros_s), float(now_monotonic_s)
        if not math.isfinite(now) or not math.isfinite(wall) or min(now, wall) < 0:
            self._raise('invalid ROS/monotonic clock')
        if self._clock_ros is not None and now < self._clock_ros-1e-9:
            self._raise('ROS clock moved backwards; explicit reset required')
        if self._clock_ros is None or now > self._clock_ros+1e-9:
            self._clock_advanced_wall = wall
        elif self.live and wall-self._clock_advanced_wall > MAX_AGE_S:
            self._raise('ROS clock paused while control session was active')
        self._clock_ros = now
        return now, wall

    def watchdog(self, *, now_ros_s, now_monotonic_s):
        if self.fault:
            raise SessionFault(self.fault)
        now, wall = self._clocks(now_ros_s, now_monotonic_s)
        if self._pending is not None and wall-self._pending.monotonic_now_s > MAX_AGE_S:
            self._raise('policy inference exceeded the publication age budget')
        if self.live:
            if self._last_success_wall is not None and wall-self._last_success_wall > MAX_AGE_S:
                self._raise('policy output stream is stale')
            for kind in KINDS:
                if not self._queues[kind]:
                    self._raise('input stream missing: %s' % kind)
                stamp, receipt, _ = self._queues[kind][-1]
                if not 0 <= wall-receipt <= MAX_AGE_S+1e-9 or not 0 <= now-stamp <= MAX_AGE_S+1e-9:
                    self._raise('stale/future ROS input stream: %s' % kind)

    def prepare(self, *, now_ros_s, now_monotonic_s):
        self.watchdog(now_ros_s=now_ros_s, now_monotonic_s=now_monotonic_s)
        if self._busy or now_ros_s <= 0:
            return None
        if self._last_start_ros is not None:
            dt = now_ros_s-self._last_start_ros
            if dt < CONTROL_DT_S-1e-6:
                return None
            if dt > CONTROL_DT_S+.025+1e-9:
                self._raise('trained 10 Hz policy period exceeded; explicit reset required')
        candidates = []
        for i, kind in enumerate(KINDS):
            rows = [r for r in self._queues[kind]
                    if 0 <= now_ros_s-r[0] <= MAX_AGE_S+1e-9
                    and 0 <= now_monotonic_s-r[1] <= MAX_AGE_S+1e-9
                    and (self._last_used is None or r[0] > self._last_used[i])]
            if not rows:
                return None
            candidates.append(rows)
        selected = None
        score = None
        for rows in itertools.product(*candidates):
            stamps = tuple(r[0] for r in rows)
            skew = max(stamps)-min(stamps)
            if skew <= MAX_SKEW_S+1e-9:
                key = (min(stamps), -skew)
                if score is None or key > score:
                    score, selected = key, rows
        if selected is None:
            return None
        self.sequence += 1
        item = WorkItem(self.epoch, self.sequence, {k: r[2] for k, r in zip(KINDS, selected)},
                        tuple(r[0] for r in selected), tuple(r[1] for r in selected),
                        float(now_ros_s), float(now_monotonic_s),
                        self._odometry_bracket(selected[0][0], now_ros_s, now_monotonic_s),
                        self._fresh_history)
        self._pending, self._busy = item, True
        self._last_start_ros = float(now_ros_s)
        return item

    def _odometry_bracket(self, rgb_t, now_ros_s, now_monotonic_s):
        """Fresh adjacent odometry samples with before.t <= rgb_t <= after.t, else None.

        The nearest-within-25 ms selection above still gates acceptance unchanged; the
        bracket only refines the pose to the RGB capture time. Samples already used by an
        earlier tick may serve as an end point. No sample newer than now exists in the
        queue, so nothing is extrapolated.
        """
        rows = [r for r in self._queues['odometry']
                if 0 <= now_ros_s-r[0] <= MAX_AGE_S+1e-9 and 0 <= now_monotonic_s-r[1] <= MAX_AGE_S+1e-9]
        before = max((r for r in rows if r[0] <= rgb_t), key=lambda r: r[0], default=None)
        after = min((r for r in rows if r[0] >= rgb_t), key=lambda r: r[0], default=None)
        if before is None or after is None or after[0]-before[0] > MAX_POSE_BRACKET_S+1e-9:
            return None
        return (before[2], after[2])

    def infer(self, item):
        """Single worker only; no callbacks/publishers/ROS clock reads here."""
        m = item.messages
        frame, state = build_frame(m['rgb'], m['depth'], m['camera_info'], m['odometry'], self.binding,
                                   odometry_bracket=item.odometry_bracket)
        pose = frame['pose_provenance']
        extra = dict(pose_interpolated=pose['pose_interpolated'], pose_skew_s=pose['pose_skew_s'],
                     coverage_case=frame['coverage_provenance']['case'])
        band = self.runtime.height_band(state)
        if not band['in_band']:
            # Outside the floor/ceiling filter band: no runtime call, no history advance, no proposal.
            return dict(extra, waiting_altitude=True, altitude=band)
        if item.fresh_history:
            self.runtime.reset()
        result = self.runtime.step(frame, state, now_s=item.ros_now_s)
        return dict(result, altitude=band, **extra)

    def finish(self, item, result, *, now_ros_s, now_monotonic_s):
        try:
            if self._pending is not item or self.fault or item.epoch != self.epoch:
                raise SessionFault(self.fault or 'obsolete inference result')
            self.watchdog(now_ros_s=now_ros_s, now_monotonic_s=now_monotonic_s)
            if (any(not 0 <= now_ros_s-t <= MAX_AGE_S+1e-9 for t in item.stamps)
                    or any(not 0 <= now_monotonic_s-t <= MAX_AGE_S+1e-9 for t in item.receipts)):
                raise SessionFault('inference result expired before publication')
            if result.get('waiting_altitude') is True:
                # A processed, on-time tick that deliberately proposes nothing: inputs are consumed
                # and the session stays live; the next in-band frame starts a fresh runtime history.
                self._last_used, self.live = item.stamps, True
                self._last_success_wall = float(now_monotonic_s)
                self.waiting_altitude, self.altitude, self._fresh_history = True, dict(result['altitude']), True
                return None
            velocity = np.asarray(result['velocity_world_enu_mps'], np.float64)
            rate = float(result['yaw_rate_rps'])
            if (velocity.shape != (3,) or not np.isfinite(velocity).all() or abs(velocity[2]) > 1e-6
                    or np.linalg.norm(velocity[:2]) > math.sqrt(2)*2.5+1e-6
                    or not math.isfinite(rate) or abs(rate) > 3.+1e-6):
                raise SessionFault('invalid/out-of-contract proposed command')
            proposal = {'capture_stamp_s': min(item.stamps), 'valid_until_s': min(item.stamps)+MAX_AGE_S,
                        'frame_id': self.binding['frames']['world'], 'velocity_enu_mps': velocity.tolist(),
                        'yaw_rate_enu_rps': rate, 'epoch': item.epoch, 'sequence': item.sequence,
                        'pose_interpolated': result.get('pose_interpolated') is True,
                        'command_sent_to_vehicle': False}
            self._last_used, self.started, self.live = item.stamps, True, True
            self._last_success_wall = float(now_monotonic_s)
            self.waiting_altitude, self._fresh_history = False, False
            self.altitude = dict(result['altitude']) if isinstance(result.get('altitude'), dict) else None
            return proposal
        except BaseException as exc:
            self.invalidate(str(exc))
            raise
        finally:
            self._pending, self._busy = None, False
            if self.fault:
                self.runtime.invalidate()

    def fail_inference(self, error):
        self._pending, self._busy = None, False
        self.invalidate('inference failed: %s' % error)
