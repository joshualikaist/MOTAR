# MOTAR camera-policy ROS2 package

This package wraps the selected MOTAR source tree. Importing it does not initialize ROS, open a camera, restore
a policy or start a flight link. The reserved `.invalid` maintainer address is an unconfigured contact placeholder.
The package has not been installed or run against a live ROS graph in this preparation.

The source path is explicit because the package does not bundle checkpoints or infer a developer's checkout.
The JSON template is `UNCONFIGURED`: its `/motar/` topics are engineering names, not discovered device or flight
controller topics. Camera alignment, mounting, timestamps and state frames must be measured and bound before
execution. The velocity output is a policy proposal; it must not be remapped directly into an autopilot input.

After installing this ament Python package in an appropriate ROS2 workspace, validate a prepared configuration:

```bash
ros2 run motar_camera_policy camera_policy \
  --source-root /absolute/path/to/MOTAR \
  --config /absolute/path/to/measured-ros2-config.json --check
```

The same metadata-only command works without a ROS installation by running `motar_camera_policy/entry.py`
with Python. `--check` does not import `rclpy` or restore a checkpoint. An unconfigured template is rejected with
status `UNCONFIGURED`; a structurally valid configuration does not establish hardware or flight qualification.

Only after configuration and the separate system checks are complete, the launch entry accepts both paths:

```bash
ros2 launch motar_camera_policy camera_policy.launch.py \
  source_root:=/absolute/path/to/MOTAR config:=/absolute/path/to/measured-ros2-config.json
```

The ROS node, session behavior and measured configuration contract are implemented in the explicitly selected
source tree's `tools/camera_ros2_node.py` and `tools/camera_ros2_session.py`. The application workflow and actual
flight-controller integration must be checked against that source and the vehicle's measured interfaces.

The second executable, `camera_px4_gateway`, loads `tools/camera_ros2_px4_node.py` in a separate process. Its
`px4_unconfigured.json` template has offboard publication disabled. Its metadata check uses the same required
`--source-root`, `--config`, `--check` arguments; `camera_px4_gateway.launch.py` likewise requires both paths.
No launch file starts a camera driver, DDS agent, simulator, or auto-arm/takeoff sequence.

## Proposed autonomy connections

| Producer | Input/output contract | Consumer |
| --- | --- | --- |
| Qualified RGB-D producer | rectified RGB, RGB-aligned optical-z depth, acquisition-stamped CameraInfo | policy node |
| PX4 estimator via uXRCE-DDS | VehicleOdometry + VehicleLocalPosition validity/reset counters + VehicleStatus | gateway |
| Gateway | private `/motar/state/odometry`, ENU pose, FLU child/body twist, original capture stamp | policy node |
| Policy | `/motar/policy/velocity_enu`, TwistStamped; oldest selected capture stamp, XY velocity and yaw rate | gateway |
| Explicitly enabled and primed gateway | PX4 OffboardControlMode + TrajectorySetpoint, NED XY velocity/current Z hold | PX4 position controller |

These topic names are our new interface design, not observations of existing hardware. No physical LiDAR,
LaserScan or simulator truth is an online input. A valid **local position estimate is still required**: for
example GNSS outdoors or separately integrated VIO/motion capture indoors. RGB-D tracking does not implement
localization. This package neither implements a VIO estimator nor feeds synthetic pose into the autopilot.

The policy consumes Odometry twist in `child_frame_id`, then rotates it once to world ENU. Camera optical
right/down/forward axes are explicitly converted to the trained camera's FLU axes using measured extrinsics.
Calibrations use rectified `CameraInfo.P`, rather than treating raw `K` as a rectified image model. Image and
CameraInfo headers must carry corresponding acquisition times; a latched CameraInfo with stamp zero is refused.
Do not repair a driver's missing capture times by stamping callbacks with the publication clock.

The gateway binds the five `.msg` definitions to the local PX4 **v1.14.3** source by SHA256. At actual node
construction it also checks the installed `px4_msgs` share directory and generated Python field types before
creating publishers. This is a source compatibility target, not a statement about the real FC version and not
a request to downgrade it. Different actual firmware needs a new reviewed binding. uXRCE-DDS already translates
PX4 timestamps to agent time; a second boot-clock offset must not be applied. Camera and ROS clocks still need
measured alignment. Use a single qualified vehicle/domain/topic namespace; this source does not identify a
vehicle by inspecting a MAVLink system ID.

Policy inference runs in one worker while the ROS executor services freshness and stop checks. Proposals expire
from their oldest capture, not their publication time. The gateway owns an independent watchdog/process. Late
inference, stale/duplicate captures, frame changes, clock discontinuities, estimator reset, failsafe or mode loss
stop publication and latch the session. Estimator reset also stops derived odometry, so a rebased pose cannot enter
old policy history. Reset the policy and restart the gateway before another explicit handover. Do not run another
offboard publisher in parallel. A hung worker may require process-supervisor termination; its executor and separate
gateway must never replay its old command to maintain offboard ownership.

The private derived Odometry contains NaN covariance entries for unavailable uncertainty, not invented zero
variance. The policy ignores covariance. This private topic is **not an estimator-fusion input**; raw PX4 variance
fields remain in the original VehicleOdometry topic and should be included in the bag.

## Physical experiment sequence to qualify separately

1. With props removed, verify actual FC/companion/ROS versions, exact `px4_msgs`, DDS topics, clock alignment,
   measured OAK mount and aligned depth. Check known translations/yaw in both ENU/FLU and NED/FRD. Configure the
   independent position estimator and verify its validity; camera-only policy does not remove this requirement.
2. Record policy proposals in observation mode. Compare a rosbag replay and actual sensor frames, target appearance,
   depth-invalid behavior and end-to-end timing. The current trained target detector is the bootstrap red rule;
   an arbitrary person/object detector has not been qualified. CPU synthetic tests do not answer these questions.
3. Use a separately reviewed and authorized Gazebo/SITL execution amendment with fresh outputs. Exercise camera,
   estimator, policy and DDS loss, process termination, reset and pilot takeover before physical policy control.
4. Qualify the basic vehicle and pilot takeover without the policy. For a policy handover, the pilot first manually
   establishes stable POSITION-mode hover. Calling `~/prime_from_manual_hover` explicitly declares this handover;
   it requires ARMED and POSITION. The code does not prove airborne state. During neutral priming it refreshes
   measured Z; it freezes Z at ACTIVE entry. After continuous priming and fresh policy output, the **pilot** selects
   OFFBOARD. There are no arm/mode/takeoff VehicleCommands. Initial physical trials need a separately declared short,
   bounded test area and duration, followed by log review before tracking or obstacles are added.
5. Record synchronized camera inputs, CameraInfo, raw PX4 estimator/status, derived state, proposals and diagnostics
   in rosbag, plus PX4 ULog and takeover events. Compare expected versus actual commands and the qualified loss
   response after every trial. A stopped stream is not a land/hover guarantee: PX4's configured offboard-loss action
   and pilot recovery must be verified on the real system.

Enabling offboard publishers requires a hash-bound flight-check record in configuration, then a separate explicit
prime service. A record's presence only binds a file; metadata validation does not certify its contents or delivery.
An operator stop revokes both offboard streams and requires a fresh policy/ownership session, never automatic re-entry.
The two DDS publications are not atomic; partial publish failure stops all subsequent publications.

## Environment status

The ament layout targets ROS 2 Humble interfaces. This host has no `/opt/ros` installation; neither colcon build,
real rclpy generated messages, a ROS graph, DDS nor a camera was exercised. The existing CPU policy environment uses
Python 3.8; Ubuntu 22.04/Humble normally uses Python 3.10. Do not load a system rclpy binary into the old conda
interpreter. The chosen ROS-compatible Python environment and policy dependencies still need a separate restore/
equivalence test, or a reviewed process boundary if those ABIs cannot coexist. No OS-wide dependency installation
or firmware modification was performed.

Primary interface references: [ROS Odometry](https://raw.githubusercontent.com/ros2/common_interfaces/humble/nav_msgs/msg/Odometry.msg),
[ROS CameraInfo](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/CameraInfo.msg),
[PX4 ROS 2 example](https://docs.px4.io/v1.14/en/ros/ros2_offboard_control),
[PX4 offboard requirements](https://docs.px4.io/v1.14/en/flight_modes/offboard),
[PX4 uXRCE-DDS](https://docs.px4.io/v1.14/en/middleware/uxrce_dds).
