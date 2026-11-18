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
stop publication and latch the session. Two deliberate exceptions do not latch a fault: altitude gating
(`WAITING_ALTITUDE`, below) proposes nothing without a fault, after which an ACTIVE gateway revokes on its stale
command; and a geofence breach latches a neutral hold with both streams kept (below). Estimator reset also stops
derived odometry, so a rebased pose cannot enter old policy history. Reset the policy and restart the gateway before another explicit handover. Do not run another
offboard publisher in parallel. A hung worker may require process-supervisor termination; its executor and separate
gateway must never replay its old command to maintain offboard ownership.

The private derived Odometry contains NaN covariance entries for unavailable uncertainty, not invented zero
variance. The policy ignores covariance. This private topic is **not an estimator-fusion input**; raw PX4 variance
fields remain in the original VehicleOdometry topic and should be included in the bag.

## Policy contract, depth semantics and height filters (fixes of 2026-11-12 and 2026-11-13)

The policy contract binds the runtime sources by SHA-256, including `tools/camera_ros2_adapter.py` and
`tools/camera_ros2_session.py`. Any source change, including these fixes, invalidates every contract generated
before it. Rebuild the contract and rebind its hash in the node configuration:

```bash
python tools/camera_policy_contract.py --output /absolute/path/to/NEW_contract.json --seed 1 \
  --height-reference-evidence /absolute/path/to/estimator_height_check.txt \
  [--ceiling-height-m MEASURED_HEIGHT] [--ground-invalid-plane-rule] [--ceiling-invalid-plane-rule]
python tools/camera_policy_contract.py --check /absolute/path/to/NEW_contract.json
sha256sum /absolute/path/to/NEW_contract.json   # -> policy_contract_sha256 in the node configuration
```

Depth semantics of the delivered frame:

- **REP 117.** 32FC1 `+Inf` means no return within range: it becomes no-return (held at MaxZ, never near).
  `-Inf`, NaN and a finite z <= 0 are sensor-invalid, and so is 16UC1 `0`. Only a dense sensor-invalid blob becomes
  a near surface (the trained `blob_near` rule); an isolated invalid pixel or any no-return pixel does not.
- **Coverage.** The node derives the undistortion coverage from CameraInfo K, D, R and P
  (`cv2.initUndistortRectifyMap`, nearest, zero border), as the OAK-D device path does, so the undistortion border is
  no-return instead of a near wall. An empty or all-zero D means full coverage. A nonzero D needs the `plumb_bob` or
  `rational_polynomial` model and OpenCV (`python3-opencv`, declared in `package.xml`); other models are refused.
  If a pixel outside the coverage carries a valid depth return, the CameraInfo does not describe the delivered image
  (for example an already undistorted image whose CameraInfo still carries D, or a different alpha or P). The frame
  is then refused and the session faults; measured depth is never erased.
- **Tilted camera.** Each trained pixel is classified by the share of sensor-invalid samples landing on it
  (>= 0.5 means sensor-invalid), so sparse valid speckle cannot hide a near surface.
- **Goal colour.** A sensor-invalid pixel that the production colour rule scores as goal-coloured becomes
  no-return and keeps its colour: it adds no phantom obstacle at the goal's bearing and cannot bias the goal range.

Height filters, contract fields `ground`, `ceiling` and `estimator_height` (the training arena had neither a floor
nor a ceiling):

- `ground`: `enabled` (default true), `margin_m` (0.10), `height_reference: "floor"`, `invalid_plane_rule` (default
  false). A valid return whose hit height lies below the margin becomes no-return.
- `ceiling`: `enabled` (default false), `height_m` (the measured room height; null while disabled), `margin_m`,
  `height_reference`, `invalid_plane_rule` (default false). Enable it with `--ceiling-height-m`.
- `invalid_plane_rule` (opt-in): also turn a sensor-invalid pixel whose ray meets the floor or ceiling plane within
  [MinZ, MaxZ] into no-return. It is off by default because it looks only at the image row and also erased a real
  obstacle closer than MinZ whose top lies a few cm below the camera. With it off, a textureless floor or ceiling
  reads as a near surface again; whether to enable it is an open item to decide from bench recordings (the OAK-D
  Pro's IR projector usually gives indoor surfaces texture).
- `estimator_height`: the operator's assertion that the estimator ENU z (the odometry position z) is the height above
  the floor, bound to an evidence file by SHA-256 through `--height-reference-evidence`, for example a record of the
  landed z within +-0.05 m of 0 and, where available, agreement with PX4 `dist_bottom`. It defaults to null and is
  never invented. While a filter is enabled without it, every frame is refused and `--check` of the node reports
  `NOT_CONFIGURED`. Building with `--no-ground-removal` instead leaves the floor in the image as an obstacle.

Altitude gating: while the camera height lies outside the enabled filter band (on the ground before take-off, or
within the ceiling margin), the session reports `WAITING_ALTITUDE` in its diagnostics, publishes no proposal and
does not advance the policy history; this is not a fault. The first in-band frame starts from a fresh history.
Input-freshness, ROS-clock and 10 Hz checks apply from the first gated tick, so a stream gap above 0.15 s while the
vehicle waits on the ground latches a fault: call `~/reset_policy_session` once the streams are steady.

## Geofence and offboard publication (PX4 gateway)

The PX4 node configuration has a required `geofence` block: `enabled`, `frame: "px4_local_enu"`, `x_min_m`,
`x_max_m`, `y_min_m`, `y_max_m`, `decel_mps2` and `latency_s`. The frame is x east, y north in metres from the PX4
EKF local-position origin, the frame of the gateway's decoded position. The box is axis-aligned in that frame, so
the operator inscribes it in the room minus a margin. An enabled fence needs the measured box (at least 0.5 m wide
on each axis), `decel_mps2` in (0, 10] m/s^2 and `latency_s` in (0, 1] s. A disabled block must leave the box and
latency null. `decel_mps2` is a horizontal deceleration norm; its template value is the trained speed governor's
braking deceleration, not a vehicle measurement.

The gateway scales the published horizontal velocity by one factor s in [0, 1], which keeps its direction: held for
`latency_s` and then braked in a straight line at `decel_mps2`, the vehicle must stop before every wall that a
velocity component points at. A component parallel to a wall therefore slows together with the normal one, as the
trained speed governor scales the whole command. Yaw rate and the altitude hold are not changed.

`latency_s` is not the transport latency. It must cover the estimator sample age, the gateway's 0.1 s publication
period and the vehicle's velocity response. In a crude closed-loop model the vehicle stayed inside only with
`latency_s` of about 2 x (vehicle response delay + 0.1 s). Measure the response delay and the achievable
deceleration on the real vehicle before enabling the fence.

A fresh position strictly outside the box latches `geofence_fault`: both streams stay up with zero XY velocity,
zero yaw rate and the altitude hold, so PX4 stays in OFFBOARD holding position. It is cleared only by the Trigger
service `~/reset_geofence` with the vehicle back inside the box (refused while outside) or by the pilot taking over;
switching to POSITION revokes the gateway. A stale policy command during a breach still revokes.

`enable_offboard_publication: true` is refused unless the geofence is enabled and valid, or the configuration
carries `geofence_waiver: {"reason": "<non-empty text>"}`, which records the operator's explicit decision to fly
without the fence. A waiver together with an enabled fence, or in an observation-only configuration, is refused.
The policy never trained with walls: leaving its 40 m arena only ended an episode.

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

Enabling offboard publishers requires a hash-bound flight-check record and an enabled geofence or an explicit
`geofence_waiver` in configuration, then a separate explicit prime service. A record's presence only binds a file; metadata validation does not certify its contents or delivery.
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
