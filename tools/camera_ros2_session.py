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

Outdoor refusals (2026-11-13, outdoor profile only): when the runtime refuses a frame with FrameRefused
(its ground plane could not be verified, or PX4's vertical validity is absent/bad), finish() returns an
explicit ZERO-velocity, zero-yaw-rate hold proposal for that tick (same stamps and freshness checks as
a policy proposal), the session reports HOLD_FRAME_REFUSED with the named reason, and the next accepted
frame starts a fresh runtime history. Without this, the first refusal faulted the session, the proposal
stream stopped, and PX4 kept executing the last nonzero velocity setpoint until its offboard-loss timeout
(COM_OF_LOSS_T, 1.0 s by default in v1.14.3). Every other InputError still faults the session as before.

Re-entry hysteresis (outdoor repair r1, review P2-1): after a refusal, the first REFUSAL_REENTRY_FRAMES accepted
frames still propose the explicit zero hold (status HOLD_REENTRY); the runtime steps on them (its fresh history
fills) and is told that the zero hold, not its command, was applied (runtime.applied_zero_hold). A nonzero command
resumes only on the next accepted frame; any refusal in between restarts the count. Intermittent refusals therefore
no longer alternate zero holds with commands from a just-reset history. The hold has NO time limit: bounding or
escalating a long hold (landing, RTL, pilot alert) is the user's decision and remains open.
Outdoor repair r2 (review r2 P3-N7): with an outdoor-profile runtime the count restarts on EVERY history reset, not
only on a refusal: reset() (session start or an explicit reset), an altitude-gated stretch (WAITING_ALTITUDE; the next
in-band frame starts a fresh history) and a refused frame each set it to REFUSAL_REENTRY_FRAMES, so a nonzero command
always follows at least that many accepted frames of the current history. Indoor contracts never refuse frames and
keep the count at 0 (their proposals are unchanged).

Bounded hold (unit B, 2026-11-18; supersedes "NO time limit" above). A hold episode is the run of consecutive ticks
whose proposal is the explicit zero hold (HOLD_FRAME_REFUSED or HOLD_REENTRY) without an intervening policy command
(camera_policy_runtime.RefusalHoldEpisode); a refusal inside it never restarts it, an altitude-gated tick (no proposal)
neither extends nor ends it, only a policy command or reset() does. Its duration is measured on the held proposals'
capture stamps. When it reaches the outdoor contract's PROVISIONAL ground_estimate.refusal_hold_max_s (default 3.0 s),
finish() proposes nothing, raises HoldTimeout and the session latches the fault `refusal_hold_timeout`: no automatic
re-entry, only an explicit reset() (a fresh session; the node's reset service) clears it. The PX4 gateway then revokes
on its existing stale-command path (no new command within 0.15 s), so PX4's offboard-loss failsafe (COM_OF_LOSS_T,
then COM_OBL_RC_ACT) and the pilot take over. No vehicle command is sent here. Indoor contracts never hold, so they
never reach it. Why capture stamps: every proposal carries one; they strictly increase from tick to tick (offer() drops
old samples, prepare() takes only newer ones); each is checked within 0.15 s of the ROS clock at prepare() and again at
finish(), and that clock is checked against monotonic time (never backwards, never paused) with the 10 Hz period
enforced on it, so ticks cannot be skipped. The replays have the same stamps and no receipt clock, so they compute the
same episode offline. On the vehicle the ROS clock is the system clock; under simulated time it is the simulator's
clock, the vehicle's own time base, where a monotonic measure would fire at a different vehicle time.

PX4 vertical fields (unit B). For an outdoor-profile runtime and a binding that declares
odometry_source = camera_ros2_adapter.gateway_odometry_source(), build_frame decodes z_valid, epv and dead_reckoning
from the gateway's own Odometry (the same message whose pose the policy uses) and attaches them to the state; absent
or malformed fields leave them out, so the runtime refuses the frame (estimator_vertical_fields_absent, a zero hold).
Indoor sessions never read them.
"""
import copy
from dataclasses import dataclass
import itertools
import math
from collections import deque

import numpy as np

from camera_ros2_adapter import (MAX_POSE_BRACKET_S, build_frame, declared_vertical_encoding, stamp_seconds,
                                 validate_binding)
# PROVISIONAL (outdoor repair r1, defined in the runtime since repair r2 so the replays share it): accepted frames held
# at zero after a history reset before a nonzero command resumes; 5 frames = 0.5 s at the trained 10 Hz, one interval
# of the trained 0.5 s observation history.
from camera_policy_contract import capture_age_bound_s
from camera_policy_runtime import (FrameRefused, REFUSAL_HOLD_TIMEOUT, REFUSAL_REENTRY_FRAMES, RefusalHoldEpisode,
                                   _is_outdoor, refusal_hold_limit_s)

KINDS = ('rgb', 'depth', 'camera_info', 'odometry')
MAX_AGE_S, MAX_SKEW_S, CONTROL_DT_S = .15, .025, .1


class SessionFault(RuntimeError):
    pass


class HoldTimeout(SessionFault):
    """The refusal-hold episode reached the contract limit; record holds the episode facts (JSON-serializable)."""
    def __init__(self, record):
        self.reason, self.record = REFUSAL_HOLD_TIMEOUT, dict(record)
        super().__init__('%s: zero-hold episode reached %.3f s (limit %.3f s, PROVISIONAL) after %d held proposals; '
                         'proposals stopped, explicit session reset required'
                         % (REFUSAL_HOLD_TIMEOUT, record['episode_s'], record['limit_s'], record['held_proposals']))


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
        # Capture ages (ROS stamps of inputs and of proposals) use the bound derived from the trained action delay;
        # receipt and cadence checks keep MAX_AGE_S (the 10 Hz stream itself must not stall).
        self.capture_age_s = capture_age_bound_s(getattr(runtime, 'contract', None))
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
        # Outdoor profile: the last tick's named refusal (a zero hold was proposed) and the run length.
        self.refusal = None
        self.consecutive_refusals = 0
        # Accepted frames still to be held at zero after the last history reset (re-entry hysteresis). Outdoor repair
        # r2 (review r2 P3-N7): an outdoor-profile session starts with the full count, as after any history reset.
        self._outdoor = _is_outdoor(getattr(self.runtime, 'contract', None))
        self.reentry_hold_remaining = REFUSAL_REENTRY_FRAMES if self._outdoor else 0
        self._reentry_holding = False
        # Unit B: bounded refusal hold (outdoor contracts only; None for indoor) and the declared PX4 vertical-field
        # encoding of the odometry input (outdoor only; indoor sessions never read it).
        self.hold_limit_s = refusal_hold_limit_s(getattr(self.runtime, 'contract', None))
        self._episode = RefusalHoldEpisode(self.hold_limit_s)
        self._episode_refusals, self._episode_last_refusal = 0, None
        self.hold_timeout = None
        self.vertical_encoding = declared_vertical_encoding(self.binding) if self._outdoor else None
        self.vertical_fields = None
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
        if self.refusal is not None:
            return 'HOLD_FRAME_REFUSED'
        if self._reentry_holding:
            return 'HOLD_REENTRY'
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
                if not 0 <= wall-receipt <= MAX_AGE_S+1e-9 or not 0 <= now-stamp <= self.capture_age_s+1e-9:
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
                    if 0 <= now_ros_s-r[0] <= self.capture_age_s+1e-9
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
                if 0 <= now_ros_s-r[0] <= self.capture_age_s+1e-9 and 0 <= now_monotonic_s-r[1] <= MAX_AGE_S+1e-9]
        before = max((r for r in rows if r[0] <= rgb_t), key=lambda r: r[0], default=None)
        after = min((r for r in rows if r[0] >= rgb_t), key=lambda r: r[0], default=None)
        if before is None or after is None or after[0]-before[0] > MAX_POSE_BRACKET_S+1e-9:
            return None
        return (before[2], after[2])

    def infer(self, item):
        """Single worker only; no callbacks/publishers/ROS clock reads here."""
        m = item.messages
        frame, state = build_frame(m['rgb'], m['depth'], m['camera_info'], m['odometry'], self.binding,
                                   odometry_bracket=item.odometry_bracket, vertical_encoding=self.vertical_encoding)
        pose = frame['pose_provenance']
        extra = dict(pose_interpolated=pose['pose_interpolated'], pose_skew_s=pose['pose_skew_s'],
                     coverage_case=frame['coverage_provenance']['case'])
        if 'vertical_provenance' in frame:
            extra['vertical_fields'] = frame['vertical_provenance']
        band = self.runtime.height_band(state)
        if not band['in_band']:
            # Outside the floor/ceiling filter band: no runtime call, no history advance, no proposal.
            return dict(extra, waiting_altitude=True, altitude=band)
        if item.fresh_history:
            self.runtime.reset()
        try:
            result = self.runtime.step(frame, state, now_s=item.ros_now_s)
        except FrameRefused as exc:
            # Outdoor profile: refused before any history advanced; finish() proposes an explicit zero hold.
            return dict(extra, frame_refused=True, refusal={'reason': exc.reason, 'detail': str(exc)}, altitude=band)
        return dict(result, altitude=band, **extra)

    def finish(self, item, result, *, now_ros_s, now_monotonic_s):
        try:
            if self._pending is not item or self.fault or item.epoch != self.epoch:
                raise SessionFault(self.fault or 'obsolete inference result')
            self.watchdog(now_ros_s=now_ros_s, now_monotonic_s=now_monotonic_s)
            if (any(not 0 <= now_ros_s-t <= self.capture_age_s+1e-9 for t in item.stamps)
                    or any(not 0 <= now_monotonic_s-t <= MAX_AGE_S+1e-9 for t in item.receipts)):
                raise SessionFault('inference result expired before publication')
            if result.get('waiting_altitude') is True:
                # A processed, on-time tick that deliberately proposes nothing: inputs are consumed
                # and the session stays live; the next in-band frame starts a fresh runtime history.
                self._last_used, self.live = item.stamps, True
                self._last_success_wall = float(now_monotonic_s)
                self.waiting_altitude, self.altitude, self._fresh_history = True, dict(result['altitude']), True
                self.refusal, self.consecutive_refusals, self._reentry_holding = None, 0, False
                if self._outdoor:
                    # Outdoor repair r2 (review r2 P3-N7): the next in-band frame starts a fresh history, so the
                    # re-entry count restarts here too.
                    self.reentry_hold_remaining = REFUSAL_REENTRY_FRAMES
                return None
            if isinstance(result.get('vertical_fields'), dict):
                self.vertical_fields = dict(result['vertical_fields'])
            if result.get('frame_refused') is True:
                # Outdoor refusal: an on-time tick whose frame was refused. Propose an explicit zero velocity and
                # zero yaw rate (the gateway keeps its altitude hold), never the previous command; the next
                # accepted frame starts a fresh runtime history.
                refusal = result.get('refusal')
                if not isinstance(refusal, dict) or not isinstance(refusal.get('reason'), str):
                    raise SessionFault('malformed frame refusal')
                # Unit B: a refusal never restarts the hold episode; at the limit nothing is proposed.
                elapsed = self._hold_tick(item, refusal['reason'])
                proposal = {'capture_stamp_s': min(item.stamps), 'valid_until_s': min(item.stamps)+self.capture_age_s,
                            'frame_id': self.binding['frames']['world'], 'velocity_enu_mps': [0., 0., 0.],
                            'yaw_rate_enu_rps': 0., 'epoch': item.epoch, 'sequence': item.sequence,
                            'pose_interpolated': result.get('pose_interpolated') is True,
                            'command_sent_to_vehicle': False, 'hold': True, 'refusal': refusal['reason'],
                            'hold_episode_s': elapsed}
                self._last_used, self.started, self.live = item.stamps, True, True
                self._last_success_wall = float(now_monotonic_s)
                self.waiting_altitude, self._fresh_history = False, True
                self.refusal, self.consecutive_refusals = dict(refusal), self.consecutive_refusals + 1
                # Re-entry hysteresis: the next REFUSAL_REENTRY_FRAMES accepted frames are held too.
                self.reentry_hold_remaining, self._reentry_holding = REFUSAL_REENTRY_FRAMES, False
                self.altitude = dict(result['altitude']) if isinstance(result.get('altitude'), dict) else None
                return proposal
            velocity = np.asarray(result['velocity_world_enu_mps'], np.float64)
            rate = float(result['yaw_rate_rps'])
            if (velocity.shape != (3,) or not np.isfinite(velocity).all() or abs(velocity[2]) > 1e-6
                    or np.linalg.norm(velocity[:2]) > math.sqrt(2)*2.5+1e-6
                    or not math.isfinite(rate) or abs(rate) > 3.+1e-6):
                raise SessionFault('invalid/out-of-contract proposed command')
            proposal = {'capture_stamp_s': min(item.stamps), 'valid_until_s': min(item.stamps)+self.capture_age_s,
                        'frame_id': self.binding['frames']['world'], 'velocity_enu_mps': velocity.tolist(),
                        'yaw_rate_enu_rps': rate, 'epoch': item.epoch, 'sequence': item.sequence,
                        'pose_interpolated': result.get('pose_interpolated') is True,
                        'command_sent_to_vehicle': False}
            self._reentry_holding = self.reentry_hold_remaining > 0
            if self._reentry_holding:
                # Unit B: a re-entry hold belongs to the hold episode too; at the limit nothing is proposed.
                elapsed = self._hold_tick(item, None)
                # Re-entry hysteresis (outdoor repair r1): an accepted frame within REFUSAL_REENTRY_FRAMES of the last
                # refusal is still held at zero; the runtime's next previous action is the applied zero hold.
                self.runtime.applied_zero_hold()
                self.reentry_hold_remaining -= 1
                proposal.update(velocity_enu_mps=[0., 0., 0.], yaw_rate_enu_rps=0., hold=True,
                                reentry_hold_remaining=self.reentry_hold_remaining, hold_episode_s=elapsed)
            else:
                # The policy's own command ends the hold episode (unit B).
                self._episode.command()
            self._last_used, self.started, self.live = item.stamps, True, True
            self._last_success_wall = float(now_monotonic_s)
            self.waiting_altitude, self._fresh_history = False, False
            self.refusal, self.consecutive_refusals = None, 0
            self.altitude = dict(result['altitude']) if isinstance(result.get('altitude'), dict) else None
            return proposal
        except BaseException as exc:
            self.invalidate(str(exc))
            raise
        finally:
            self._pending, self._busy = None, False
            if self.fault:
                self.runtime.invalidate()

    def _hold_tick(self, item, refusal_reason):
        """Count one zero-hold proposal in the current episode; at the limit latch refusal_hold_timeout (unit B)."""
        capture = min(item.stamps)
        if not self._episode.active:
            self._episode_refusals, self._episode_last_refusal = 0, None
        elapsed, timed_out = self._episode.hold(capture)
        if refusal_reason is not None:
            self._episode_refusals += 1
            self._episode_last_refusal = refusal_reason
        if timed_out:
            self.hold_timeout = {'reason': REFUSAL_HOLD_TIMEOUT, 'episode_s': elapsed, 'limit_s': self.hold_limit_s,
                                 'limit_status': 'PROVISIONAL', 'first_hold_capture_s': self._episode.start_s,
                                 'capture_stamp_s': capture, 'held_proposals': self._episode.ticks - 1,
                                 'refused_frames': self._episode_refusals, 'last_refusal': self._episode_last_refusal,
                                 'epoch': item.epoch, 'sequence': item.sequence}
            raise HoldTimeout(self.hold_timeout)
        return elapsed

    @property
    def hold_episode_s(self):
        """Duration of the current refusal-hold episode at its last held proposal; None outside an episode."""
        return self._episode.elapsed_s

    def diagnostics(self):
        """JSON-serializable session state for the node's diagnostic topic (no node-local fields)."""
        return {'state': self.status, 'fault': self.fault, 'altitude': self.altitude, 'epoch': self.epoch,
                'sequence': self.sequence, 'vehicle_commands_sent': 0,
                # Outdoor profile: the last named frame refusal (a zero hold was proposed) and its run length.
                'refusal': self.refusal, 'consecutive_refusals': self.consecutive_refusals,
                'reentry_hold_remaining': self.reentry_hold_remaining,
                # Unit B: the bounded hold (PROVISIONAL limit) and the PX4 vertical fields of the last tick.
                'hold_episode_s': self.hold_episode_s, 'hold_limit_s': self.hold_limit_s,
                'hold_limit_status': None if self.hold_limit_s is None else 'PROVISIONAL',
                'hold_timeout': self.hold_timeout, 'vertical_encoding': self.vertical_encoding,
                'vertical_fields': self.vertical_fields}

    def fail_inference(self, error):
        self._pending, self._busy = None, False
        self.invalidate('inference failed: %s' % error)
