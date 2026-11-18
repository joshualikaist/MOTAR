"""ROS 2 autonomy policy node: calibrated RGB-D + odometry -> ENU proposal.

`--check` through the ament entry point is metadata-only and needs no ROS install.
The actual node is imported/constructed only by main(check_only=False). It never
arms a vehicle or publishes PX4 messages. Its stamped velocity proposals are for
a separate, explicitly enabled offboard owner/watchdog.

ROS 2 Humble source APIs:
https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py
https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos.py
https://github.com/ros2/common_interfaces/tree/humble
"""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys
import time


def load_node_config(config_path, source_root):
    root = Path(source_root).resolve()
    if not Path(source_root).is_absolute() or not (root/'tools/camera_policy_runtime.py').is_file():
        raise ValueError('explicit absolute policy source root is required')
    tools = root/'tools'
    for name in ('camera_ros2_adapter', 'camera_ros2_session', 'camera_policy_contract', 'camera_policy_runtime'):
        loaded = sys.modules.get(name)
        if loaded is not None and Path(loaded.__file__).resolve().parent != tools:
            raise ValueError('previously imported module belongs to another source root: %s' % name)
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    config_path = Path(config_path).resolve()
    raw = json.loads(config_path.read_text())
    expected = {'format', 'binding', 'policy_contract_path', 'policy_contract_sha256', 'outputs'}
    if set(raw) != expected or raw['format'] != 'motar-ros2-node-v1':
        raise ValueError('unsupported ROS node configuration schema')
    if raw['binding'].get('status') != 'CONFIGURED':
        raise ValueError('UNCONFIGURED: measured geometry, aligned producer and clock binding are required')
    adapter = importlib.import_module('camera_ros2_adapter')
    raw['binding'] = adapter.validate_binding(raw['binding'])
    outputs = raw['outputs']
    if not isinstance(outputs, dict) or set(outputs) != {'velocity_topic', 'diagnostic_topic'}:
        raise ValueError('explicit proposal and diagnostic topics required')
    for topic in outputs.values():
        if (not isinstance(topic, str) or not topic.startswith('/motar/') or '//' in topic
                or any(c.isspace() for c in topic) or topic.endswith('/')):
            raise ValueError('policy output must use an explicit /motar/ proposal namespace')
    if len(set(outputs.values())) != 2 or set(outputs.values()) & set(raw['binding']['topics'].values()):
        raise ValueError('ROS input/output topics must be distinct')
    if not isinstance(raw['policy_contract_path'], str) or not raw['policy_contract_path']:
        raise ValueError('policy contract path is required')
    path = Path(raw['policy_contract_path'])
    if not path.is_absolute():
        path = config_path.parent/path
    path = path.resolve()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if raw['policy_contract_sha256'] != digest:
        raise ValueError('policy contract file differs from its binding')
    contract_module = importlib.import_module('camera_policy_contract')
    contract = contract_module.load_contract(path, root=root)
    # Repair r1 (F6): an enabled floor/ceiling filter needs the operator's estimator-height assertion;
    # refuse here so the node never starts a session whose every frame would be refused.
    problem = contract_module.height_reference_problem(contract)
    if problem:
        raise ValueError('UNCONFIGURED: ' + problem)
    raw['policy_contract_path'] = str(path)
    return raw, contract


def stamp_parts(seconds):
    value = float(seconds)
    if not math.isfinite(value) or value < 0:
        raise ValueError('invalid capture timestamp')
    total = int(round(value*1e9))
    return divmod(total, 1000000000)


def make_node_class():
    """Lazy ROS imports. Calling this function itself requires an installed ROS 2."""
    from rclpy.node import Node
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from camera_ros2_session import RosPolicySession, SessionFault

    class CameraPolicyNode(Node):
        def __init__(self, config, runtime):
            super().__init__('motar_camera_policy')
            self._config = config
            self._session = RosPolicySession(runtime, config['binding'])
            self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='camera_policy')
            self._future, self._work, self._closed = None, None, False
            self._last_error = None
            self._last_diag_wall = -float('inf')
            sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            command_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                lifespan=Duration(seconds=.15))
            self._velocity = self.create_publisher(TwistStamped, config['outputs']['velocity_topic'], command_qos)
            self._diagnostics = self.create_publisher(String, config['outputs']['diagnostic_topic'], 1)
            types = {'rgb': Image, 'depth': Image, 'camera_info': CameraInfo, 'odometry': Odometry}
            self._subscriptions = [self.create_subscription(types[k], config['binding']['topics'][k],
                                      lambda msg, kind=k: self._receive(kind, msg), sensor_qos) for k in types]
            self._reset_service = self.create_service(Trigger, '~/reset_policy_session', self._reset)
            # Independent of /clock pauses: stale inference never keeps publishing.
            self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self._timer = self.create_timer(.02, self._poll, clock=self._steady_clock)

        def _receive(self, kind, msg):
            try:
                self._session.offer(kind, msg, received_monotonic_s=time.monotonic())
            except Exception as exc:
                self._session.invalidate(str(exc))
                self._last_error = str(exc)

        def _reset(self, request, response):
            if self._future is not None:
                response.success, response.message = False, 'wait for inference completion before resetting'
            else:
                self._session.reset()
                self._last_error = None
                response.success, response.message = True, 'policy session reset; waiting for fresh aligned inputs'
            return response

        def _publish_proposal(self, proposal):
            message = TwistStamped()
            sec, ns = stamp_parts(proposal['capture_stamp_s'])
            message.header.stamp.sec, message.header.stamp.nanosec = sec, ns
            message.header.frame_id = proposal['frame_id']
            message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = proposal['velocity_enu_mps']
            message.twist.angular.z = proposal['yaw_rate_enu_rps']
            self._velocity.publish(message)

        def _poll(self):
            if self._closed:
                return
            now = self.get_clock().now().nanoseconds/1e9
            wall = time.monotonic()
            try:
                self._session.watchdog(now_ros_s=now, now_monotonic_s=wall)
            except SessionFault as exc:
                self._last_error = str(exc)
            if self._future is not None and self._future.done():
                try:
                    result = self._future.result()
                    proposal = self._session.finish(self._work, result,
                        now_ros_s=self.get_clock().now().nanoseconds/1e9, now_monotonic_s=time.monotonic())
                    # None: an altitude-gated tick (WAITING_ALTITUDE) that proposes nothing.
                    if proposal is not None and not self._closed:
                        self._publish_proposal(proposal)
                except Exception as exc:
                    self._session.fail_inference(exc)
                    self._last_error = str(exc)
                finally:
                    self._future, self._work = None, None
            if self._future is None and not self._session.fault:
                try:
                    item = self._session.prepare(now_ros_s=self.get_clock().now().nanoseconds/1e9,
                                                 now_monotonic_s=time.monotonic())
                    if item is not None:
                        self._work = item
                        self._future = self._pool.submit(self._session.infer, item)
                except Exception as exc:
                    self._session.invalidate(str(exc))
                    self._last_error = str(exc)
            if wall-self._last_diag_wall >= 1.:
                msg = String()
                msg.data = json.dumps({'state': self._session.status, 'altitude': self._session.altitude,
                    'last_error': self._last_error, 'epoch': self._session.epoch,
                    'sequence': self._session.sequence, 'vehicle_commands_sent': 0}, allow_nan=False)
                self._diagnostics.publish(msg)
                self._last_diag_wall = wall

        def destroy_node(self):
            self._closed = True
            self._session.invalidate('ROS policy node shutdown')
            if self._future is not None:
                self._future.cancel()
            self._pool.shutdown(wait=False)
            return super().destroy_node()

    return CameraPolicyNode


def main(ros_args=None, *, config_path, source_root, check_only=False):
    try:
        config, contract = load_node_config(config_path, source_root)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        if check_only:
            print(json.dumps({'status': 'NOT_CONFIGURED', 'reason': str(exc),
                              'ros_graph': 'NOT_STARTED', 'hardware': 'NOT_TESTED'}))
            return 2
        raise
    if check_only:
        print(json.dumps({'status': 'CONFIG_VALID', 'lineage': contract['lineage'], 'seed': contract['seed'],
                          'ros_graph': 'NOT_STARTED', 'hardware': 'NOT_TESTED'}))
        return 0
    # Policy restore happens before creating ROS entities or any offboard stream.
    from camera_policy_runtime import CameraPolicyRuntime
    runtime = CameraPolicyRuntime(contract)
    import rclpy
    node = None
    try:
        rclpy.init(args=ros_args)
        node = make_node_class()(config, runtime)
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
