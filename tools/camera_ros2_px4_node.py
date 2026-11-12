"""Explicit ROS 2/PX4 bridge. Default observation only; no arm, mode or takeoff commands.

The installed px4_msgs definitions must equal the bound firmware source. Neither
schema checks nor generated messages establish delivery, actual FC compatibility
or flight qualification. ROS imports occur only after explicit configuration.
"""
import copy
import hashlib
import importlib
import json
import math
from pathlib import Path
import re
import sys
import time


def unconfigured_template():
    return dict(format='motar-ros2-px4-node-v1', status='UNCONFIGURED',
        px4_msgs_msg_dir=None, firmware=dict(version='v1.14.3', source=None),
        frames=dict(world=None, body=None),
        topics=dict(velocity='/motar/policy/velocity_enu', odometry='/motar/state/odometry',
                    diagnostics='/motar/px4/diagnostics'),
        enable_offboard_publication=False, flight_checks_provenance=None)


def verify_message_definitions(directory):
    from camera_ros2_px4 import PX4_MESSAGE_SHA256
    path = Path(directory)
    if not path.is_absolute():
        raise ValueError('explicit absolute px4_msgs msg directory required')
    for name, digest in PX4_MESSAGE_SHA256.items():
        if hashlib.sha256((path/name).read_bytes()).hexdigest() != digest:
            raise ValueError('px4_msgs definition differs from bound PX4 v1.14.3: '+name)
    return path.resolve()


def validate_generated_type(cls, definition):
    """Bind the generated Python fields to the already SHA-checked .msg source."""
    aliases = {'float32': 'float', 'float64': 'double', 'bool': 'boolean'}
    expected = {}
    for line in Path(definition).read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if not line or '=' in line:
            continue
        match = re.fullmatch(r'(\w+)(\[\d+\])?\s+(\w+)', line)
        if match is None:
            raise ValueError('unsupported bound message declaration: '+line)
        kind, array, name = match.groups()
        expected[name] = aliases.get(kind, kind)+(array or '')
    actual = cls.get_fields_and_field_types()
    actual = {name: aliases.get(kind, kind) for name, kind in actual.items()}
    if not expected or actual != expected:
        raise ValueError('generated px4_msgs fields differ from source: '+Path(definition).name)


def load_node_config(config_path, source_root):
    root = Path(source_root).resolve()
    if not Path(source_root).is_absolute() or not (root/'tools/camera_ros2_px4.py').is_file():
        raise ValueError('explicit absolute PX4 source root required')
    for name in ('camera_ros2_px4', 'camera_ros2_px4_node', 'camera_ros2_node'):
        loaded = sys.modules.get(name)
        if loaded is not None and Path(loaded.__file__).resolve().parent != root/'tools':
            raise ValueError('module already imported from another source root: '+name)
    if str(root/'tools') not in sys.path:
        sys.path.insert(0, str(root/'tools'))
    raw = json.loads(Path(config_path).read_text())
    if set(raw) != set(unconfigured_template()) or raw['format'] != 'motar-ros2-px4-node-v1':
        raise ValueError('unsupported PX4 node configuration schema')
    if raw['status'] != 'CONFIGURED':
        raise ValueError('UNCONFIGURED: explicit frames, message-build and clock provenance required')
    if set(raw['frames']) != {'world', 'body'} or len(set(raw['frames'].values())) != 2:
        raise ValueError('distinct ENU world and FLU body frames required')
    for frame in raw['frames'].values():
        if not isinstance(frame, str) or not frame or frame.startswith('/') or any(c.isspace() for c in frame):
            raise ValueError('invalid frame name')
    if set(raw['topics']) != {'velocity', 'odometry', 'diagnostics'} or len(set(raw['topics'].values())) != 3:
        raise ValueError('distinct proposal, odometry and diagnostic topics required')
    for topic in raw['topics'].values():
        if (not isinstance(topic, str) or not topic.startswith('/motar/') or '//' in topic
                or topic.endswith('/') or any(c.isspace() for c in topic)):
            raise ValueError('bridge topics must use explicit /motar/ namespace')
    if type(raw['enable_offboard_publication']) is not bool:
        raise ValueError('enable_offboard_publication must be boolean')
    if set(raw['firmware']) != {'version', 'source'}:
        raise ValueError('explicit target firmware declaration required')
    from camera_ros2_px4 import Px4MessageBinding, PX4_MESSAGE_SHA256
    binding = Px4MessageBinding(raw['firmware']['version'], PX4_MESSAGE_SHA256, raw['firmware']['source'])
    raw['px4_msgs_msg_dir'] = str(verify_message_definitions(raw['px4_msgs_msg_dir']))
    proof = raw['flight_checks_provenance']
    if raw['enable_offboard_publication']:
        if not isinstance(proof, dict) or set(proof) != {'path', 'sha256'}:
            raise ValueError('enabled publication requires explicit flight-check record binding')
        path = Path(proof['path'])
        if not path.is_absolute() or hashlib.sha256(path.read_bytes()).hexdigest() != proof['sha256']:
            raise ValueError('flight-check record differs from binding')
    elif proof is not None:
        raise ValueError('observation template must leave flight-check record unbound')
    return raw, binding


class BridgeSession:
    """Serialized receive/dispatch state, CPU testable without constructing ROS entities.

    An estimator reset stops odometry as well as both offboard streams. The policy
    then loses input freshness and latches; both processes must be reset/restarted.
    No rebased pose is exposed to a policy that still has pre-reset history.
    """
    def __init__(self, config, binding):
        from camera_ros2_px4 import Px4OffboardGateway
        self.config = copy.deepcopy(config)
        self.gate = Px4OffboardGateway(binding, command_frame_id=config['frames']['world'])
        self.messages, self.receipts, self.identities = {}, {}, {}
        self.fault = None
        self.command = self.command_receipt = None
        self.resets = self.last_odom = self.last_tick = self.last_clocks = None

    def revoke(self, reason):
        if self.fault is None:
            self.fault = str(reason)
        self.gate.revoke(self.fault)

    def offer(self, kind, message, *, receipt):
        if self.fault:
            return False
        if kind not in ('odometry', 'local_position', 'status'):
            raise ValueError('unknown PX4 input')
        stamp = message['timestamp' if kind == 'status' else 'timestamp_sample']
        if type(stamp) is not int or stamp <= 0 or not math.isfinite(receipt) or receipt < 0:
            self.revoke('invalid PX4 sample identity/receipt')
            return False
        # Publication time may advance while the capture remains identical.
        # Re-delivery never renews either receipt age or the original payload.
        signature = json.dumps({k: v for k, v in message.items() if k != 'timestamp'}, sort_keys=True)
        if kind in self.identities:
            old_stamp, old_signature = self.identities[kind]
            if stamp < old_stamp or (stamp == old_stamp and signature != old_signature):
                self.revoke('PX4 capture regressed or changed payload: '+kind)
                return False
            if stamp == old_stamp:
                return False
        self.messages[kind], self.receipts[kind] = copy.deepcopy(message), float(receipt)
        self.identities[kind] = (stamp, signature)
        return True

    def offer_command(self, command, *, receipt):
        if self.fault:
            return
        # Keep the first receipt of duplicate proposals, including before prime.
        if self.command is not None and command['capture_stamp_s'] == self.command['capture_stamp_s']:
            if command != self.command:
                self.revoke('policy payload changed at the same capture')
            return
        if self.command is not None and command['capture_stamp_s'] < self.command['capture_stamp_s']:
            self.revoke('policy capture regressed')
            return
        self.command, self.command_receipt = copy.deepcopy(command), float(receipt)

    def snapshot(self):
        if len(self.messages) != 3:
            raise ValueError('waiting for PX4 odometry, local position and status')
        return dict(self.messages, receipts=dict(self.receipts))

    def _state(self, now, wall):
        from camera_ros2_px4 import decode_estimator, RESET_FIELDS
        snapshot = self.snapshot()
        state = decode_estimator(snapshot['odometry'], snapshot['local_position'], snapshot['status'],
                                 now_s=now, now_monotonic_s=wall, receipts=snapshot['receipts'])
        resets = tuple(snapshot['local_position'][k] for k in RESET_FIELDS)+(snapshot['odometry']['reset_counter'],)
        if self.resets is not None and resets != self.resets:
            raise ValueError('estimator reset; restart gateway and reset policy before rebinding')
        self.resets = resets
        return state

    def prime(self, *, now, wall):
        from camera_ros2_px4 import ARMED, POSITION
        try:
            if self.fault or not self.config['enable_offboard_publication']:
                raise ValueError(self.fault or 'observation only; offboard publishers disabled')
            self._state(now, wall)
            # Calling prime_from_manual_hover is the operator's explicit handover.
            # The software cannot infer airborne/stable hover from an armed bit.
            if self.messages['status']['arming_state'] != ARMED or self.messages['status']['nav_state'] != POSITION:
                raise ValueError('prime requires manually armed PX4 POSITION mode hover')
            payload = self.gate.prime(self.snapshot(), now_s=now, now_monotonic_s=wall,
                                      manual_hover_handover=True)
            self.last_tick = wall
            return payload
        except Exception as exc:
            self.revoke(str(exc))
            raise

    def poll(self, *, now, wall):
        """Return (new measured state or None, new offboard pair or None)."""
        if self.fault:
            return None, None
        try:
            if not math.isfinite(now) or not math.isfinite(wall) or min(now, wall) < 0:
                raise ValueError('invalid ROS/monotonic clock')
            if self.last_clocks is not None:
                ros_dt, wall_dt = now-self.last_clocks[0], wall-self.last_clocks[1]
                if min(ros_dt, wall_dt) < -1e-9 or abs(ros_dt-wall_dt) > .025+1e-9:
                    raise ValueError('ROS/monotonic clock jump or pause')
            self.last_clocks = (now, wall)
            if len(self.messages) < 3:
                return None, None
            state = self._state(now, wall)
            if self.last_odom == state['t']:
                state = None
            else:
                self.last_odom = state['t']
            payload = None
            if self.gate.state not in ('DISABLED', 'REVOKED') and (self.last_tick is None or wall-self.last_tick >= .1-1e-9):
                payload = self.gate.update(self.snapshot(), self.command,
                    command_received_monotonic_s=self.command_receipt, now_s=now, now_monotonic_s=wall)
                self.last_tick = wall
                if payload is None:
                    raise ValueError(self.gate.reason or 'offboard ownership revoked')
            return state, payload
        except Exception as exc:
            self.revoke(str(exc))
            return None, None


def message_mapping(message):
    result = {}
    for name in message.get_fields_and_field_types():
        value = getattr(message, name)
        result[name] = value.tolist() if hasattr(value, 'tolist') else list(value) if isinstance(value, tuple) else value
    return result


def make_node_class():
    import numpy as np
    from rclpy.node import Node
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleStatus, OffboardControlMode, TrajectorySetpoint
    from camera_ros2_px4 import TOPICS
    from camera_ros2_adapter import stamp_seconds
    from camera_ros2_node import stamp_parts

    class Px4GatewayNode(Node):
        def __init__(self, config, binding):
            # Check the definitions of the package actually imported, not only a
            # user-selected firmware source directory. Never create publishers first.
            definitions = verify_message_definitions(Path(get_package_share_directory('px4_msgs'))/'msg')
            for cls in (VehicleOdometry, VehicleLocalPosition, VehicleStatus, OffboardControlMode, TrajectorySetpoint):
                validate_generated_type(cls, definitions/(cls.__name__+'.msg'))
            super().__init__('motar_px4_gateway')
            self._session = BridgeSession(config, binding)
            self._closed = False
            self._last_diag = -float('inf')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self._odom = self.create_publisher(Odometry, config['topics']['odometry'], qos)
            self._diag = self.create_publisher(String, config['topics']['diagnostics'], 1)
            self._publishers = {}
            if config['enable_offboard_publication']:
                # Reliable publisher matches both PX4's best-effort and reliable
                # subscribers; volatile, depth1 avoids retained stale commands.
                output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE, lifespan=Duration(seconds=.15))
                self._publishers = {key: self.create_publisher(cls, TOPICS[key], output_qos) for key, cls in
                    (('offboard_control_mode', OffboardControlMode), ('trajectory_setpoint', TrajectorySetpoint))}
            self._subs = [self.create_subscription(cls, TOPICS[k], lambda msg, kind=k: self._receive(kind, msg), qos)
                for k, cls in (('odometry', VehicleOdometry), ('local_position', VehicleLocalPosition), ('status', VehicleStatus))]
            self._subs.append(self.create_subscription(TwistStamped, config['topics']['velocity'], self._command, qos))
            self._prime_srv = self.create_service(Trigger, '~/prime_from_manual_hover', self._prime)
            self._stop_srv = self.create_service(Trigger, '~/stop', self._stop)
            self._clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._timer = self.create_timer(.02, self._poll, clock=self._clock)

        def _receive(self, kind, message):
            try:
                self._session.offer(kind, message_mapping(message), receipt=time.monotonic())
            except Exception as exc:
                self._session.revoke(str(exc))

        def _command(self, message):
            try:
                if message.twist.angular.x != 0 or message.twist.angular.y != 0:
                    raise ValueError('roll/pitch rate proposals unsupported')
                v = message.twist.linear
                self._session.offer_command(dict(capture_stamp_s=stamp_seconds(message.header.stamp),
                    frame_id=message.header.frame_id, velocity_enu_mps=[v.x, v.y, v.z],
                    yaw_rate_enu_rps=message.twist.angular.z), receipt=time.monotonic())
            except Exception as exc:
                self._session.revoke(str(exc))

        def _send(self, payload):
            if payload is None or self._closed or self._session.fault:
                return
            try:
                # Construct/validate both typed messages before sending either.
                messages = {}
                for key, cls in (('offboard_control_mode', OffboardControlMode), ('trajectory_setpoint', TrajectorySetpoint)):
                    message = cls()
                    for name, value in payload[key].items():
                        setattr(message, name, value)
                    messages[key] = message
                for key in ('trajectory_setpoint', 'offboard_control_mode'):
                    self._publishers[key].publish(messages[key])
            except Exception as exc:
                # DDS sends cannot be atomic. A partial publish fault revokes all
                # subsequent streams; actual FC offboard-loss behavior is a live gate.
                self._session.revoke('PX4 publication failed: '+str(exc))

        def _prime(self, request, response):
            try:
                self._send(self._session.prime(now=self.get_clock().now().nanoseconds/1e9, wall=time.monotonic()))
                if self._session.fault:
                    raise ValueError(self._session.fault)
                response.success, response.message = True, 'neutral priming; operator owns manual OFFBOARD handover after 1.1s'
            except Exception as exc:
                response.success, response.message = False, str(exc)
            return response

        def _stop(self, request, response):
            self._session.revoke('operator stopped offboard ownership')
            response.success, response.message = True, 'streams stopped; pilot/qualified PX4 loss response owns recovery'
            return response

        def _publish_odometry(self, state):
            message = Odometry()
            sec, ns = stamp_parts(state['t'])
            message.header.stamp.sec, message.header.stamp.nanosec = sec, ns
            message.header.frame_id = self._session.config['frames']['world']
            message.child_frame_id = self._session.config['frames']['body']
            p, q = state['pos'], state['quat_xyzw']
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = p
            (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
             message.pose.pose.orientation.z, message.pose.pose.orientation.w) = q
            # nav_msgs twist is child/body frame. PX4's NED world velocity has
            # already become ENU; rotate exactly once into FLU for ROS consumers.
            x, y, z, w = q
            r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                          [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                          [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            v = r.T @ np.asarray(state['vel'])
            message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = map(float, v)
            rate = self._session.messages['odometry']['angular_velocity']
            message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = rate[0], -rate[1], -rate[2]
            # Unknown cross-covariance/angular-rate uncertainty is not a measured
            # zero. Private policy input only: NaNs explicitly mean unavailable;
            # this Odometry must not be reused as an estimator-fusion input.
            message.pose.covariance = [float('nan')]*36
            message.twist.covariance = [float('nan')]*36
            self._odom.publish(message)

        def _poll(self):
            if self._closed:
                return
            wall, now = time.monotonic(), self.get_clock().now().nanoseconds/1e9
            state, payload = self._session.poll(now=now, wall=wall)
            try:
                if state is not None:
                    self._publish_odometry(state)
                self._send(payload)
            except Exception as exc:
                self._session.revoke('ROS publication failed: '+str(exc))
            if wall-self._last_diag >= 1.:
                message = String()
                message.data = json.dumps(dict(state=self._session.gate.state, fault=self._session.fault,
                    mode='OFFBOARD_CAPABLE' if self._publishers else 'OBSERVATION_ONLY',
                    hardware='NOT_TESTED', dds_delivery='NOT_TESTED'))
                self._diag.publish(message)
                self._last_diag = wall

        def destroy_node(self):
            self._closed = True
            self._session.revoke('PX4 bridge shutdown')
            return super().destroy_node()

    return Px4GatewayNode


def main(ros_args=None, *, config_path, source_root, check_only=False):
    try:
        config, binding = load_node_config(config_path, source_root)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        if check_only:
            print(json.dumps(dict(status='NOT_CONFIGURED', reason=str(exc), ros_graph='NOT_STARTED', hardware='NOT_TESTED')))
            return 2
        raise
    if check_only:
        print(json.dumps(dict(status='CONFIG_VALID', mode='OFFBOARD_CAPABLE' if config['enable_offboard_publication']
                        else 'OBSERVATION_ONLY', binding=binding.metadata(), ros_graph='NOT_STARTED')))
        return 0
    import rclpy
    node = None
    try:
        rclpy.init(args=ros_args)
        node = make_node_class()(config, binding)
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
