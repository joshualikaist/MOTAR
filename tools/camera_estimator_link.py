"""Atomic, attributed PX4 estimator snapshots for the camera-only companion.

The caller owns an explicitly opened pymavlink connection and its receive loop.
Importing/constructing this adapter opens no port and sends no command. Position,
attitude and heartbeat receipt times use one supplied monotonic clock. HIL state
and all target truth messages are ignored. Publication is opt-in; arming and mode
changes are deliberately outside this adapter's interface.
"""
import math
import threading
import time


class EstimatorLinkError(RuntimeError):
    pass


class EstimatorLink:
    def __init__(self, connection, *, system_id, component_id, publish=False,
                 clock=time.monotonic, mav_frame_local_ned=1):
        if (type(system_id) is not int or not 1 <= system_id <= 255
                or type(component_id) is not int or not 1 <= component_id <= 255
                or type(publish) is not bool or not callable(clock)
                or mav_frame_local_ned != 1):
            raise ValueError('explicit PX4 source IDs, publish flag, monotonic clock and LOCAL_NED frame required')
        self.connection, self.system_id, self.component_id = connection, system_id, component_id
        self.publish, self.clock = publish, clock
        self._mu, self._send_mu = threading.Lock(), threading.Lock()
        self._state = dict(pos=None, att=None, hb=None, pos_wall_t=None, att_wall_t=None, hb_wall_t=None)
        self._fault = None
        self.proposals = []

    def accept(self, message):
        """Accept a source-matched decoded message; no simulator-only message is read."""
        if message.get_srcSystem() != self.system_id or message.get_srcComponent() != self.component_id:
            return False
        kind = message.get_type()
        if kind not in ('LOCAL_POSITION_NED', 'ATTITUDE', 'HEARTBEAT'):
            return False
        received = float(self.clock())
        if not math.isfinite(received) or received < 0:
            raise EstimatorLinkError('invalid monotonic receipt time')
        if kind == 'LOCAL_POSITION_NED':
            key = 'pos'
            value = (message.time_boot_ms*.001, message.x, message.y, message.z,
                     message.vx, message.vy, message.vz)
        elif kind == 'ATTITUDE':
            key = 'att'
            value = (message.time_boot_ms*.001, message.roll, message.pitch, message.yaw,
                     message.rollspeed, message.pitchspeed, message.yawspeed)
        else:
            key, value = 'hb', (message.base_mode, message.custom_mode)
        with self._mu:
            if self._fault:
                raise EstimatorLinkError(self._fault)
            if (not all(isinstance(x, (int, float)) and math.isfinite(x) for x in value)
                    or (key == 'hb' and any(type(x) is not int for x in value))):
                self._fault = 'malformed/nonfinite PX4 %s' % kind
                raise EstimatorLinkError(self._fault)
            old = self._state[key]
            # A delayed duplicate is not a new sample; do not refresh its receipt
            # time. A boot clock regression/reboot invalidates this whole session.
            if key != 'hb' and old is not None:
                if value[0] < old[0]:
                    self._fault = 'PX4 boot clock regressed; new session/clock binding required'
                    raise EstimatorLinkError(self._fault)
                if value[0] == old[0]:
                    return False
            self._state[key], self._state[key+'_wall_t'] = tuple(value), received
        return True

    def poll(self, *, timeout_s=.025):
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= .025:
            raise ValueError('poll timeout must be in (0, 0.025] seconds')
        message = self.connection.recv_match(blocking=True, timeout=timeout_s)
        return False if message is None else self.accept(message)

    def snapshot(self):
        with self._mu:
            if self._fault:
                raise EstimatorLinkError(self._fault)
            return dict(self._state)

    def setpoint(self, mask, x=0., y=0., z=0., vx=0., vy=0., yaw_rate=0.):
        """Record a proposal; publish only if explicitly enabled and current state is valid."""
        from sitl_camera import ClockBinding, estimator_state, TM_POS_Z_VEL_XY_YAWRATE
        values = (x, y, z, vx, vy, yaw_rate)
        if (type(mask) is not int or mask != TM_POS_Z_VEL_XY_YAWRATE
                or not all(math.isfinite(v) for v in values)
                or z >= 0 or math.hypot(vx, vy) > math.sqrt(2.)*2.5+1e-6 or abs(yaw_rate) > 3.+1e-6):
            raise EstimatorLinkError('invalid camera velocity/altitude setpoint')
        # This checks local receipt ages/modes only. The camera driver separately
        # requires the measured boot-to-sensor binding for capture alignment.
        estimator_state(self, ClockBinding(0., 'receipt/mode validation only; no sensor timestamp compared'),
                        clock=self.clock)
        proposal = {'mask': mask, 'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy,
                    'yaw_rate': yaw_rate, 'published': False}
        if self.publish:
            with self._send_mu:
                estimator_state(self, ClockBinding(0., 'receipt/mode validation only; no sensor timestamp compared'),
                                clock=self.clock)
                self.connection.mav.set_position_target_local_ned_send(
                    0, self.system_id, self.component_id, 1, mask,
                    x, y, z, vx, vy, 0., 0., 0., 0., 0., yaw_rate)
                proposal['published'] = True
        self.proposals.append(proposal)
        return proposal
