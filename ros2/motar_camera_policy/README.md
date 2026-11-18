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

## Outdoor profile (2026-11-13, opt-in)

Outdoors the OAK-D reports open sky as depth 0 (no stereo match). With the indoor contract the runtime maps 0 to
sensor-invalid and blob_near turns the dense sky into a 0.4 m surface on every bearing, while the policy was trained
with the sky as no-return (MaxZ) and with neither floor nor ceiling. The outdoor profile is opt-in; every contract
built without it behaves exactly as before.

```bash
python -B tools/camera_policy_contract.py --output outdoor_contract.json --seed 1 --profile outdoor
```

`--profile outdoor` writes `profile: "outdoor"`, the floor filter on (plane rule off), a virtual ceiling at 3.0 m
(`ceiling.physical: false`; `--ceiling-height-m` overrides within [2.5, 6.0] m, PROVISIONAL bounds), and two new
blocks, `sky` and `ground_estimate`. It refuses `--no-ground-removal`, either invalid plane rule and
`--height-reference-evidence`. The virtual ceiling removes valid returns above it and touches nothing else: it never
explains a sensor-invalid pixel (`physical: false` with `invalid_plane_rule: true` is refused) and never connects the
sky rule. `ceiling.physical` is accepted only together with the outdoor profile; `refuse_dead_reckoning` must be true.

**Ground verified per frame (`estimate_ground`).** PX4's z is relative to the take-off point and drifts by
decimetres, so it is not trusted alone and the operator floor assertion (`estimator_height`) is not used outdoors;
it must stay null. On every frame the runtime fits the ground from the camera: valid source pixels whose ray points
below the horizon within `fit_max_range_m`, in the gravity frame given by PX4's attitude, deterministic sequential
RANSAC (fixed seed, MSAC score) with total-least-squares refinement. It reports the camera height above the plane,
the tilt, inlier count and share, residual RMS and the share of points below the plane. The frame is accepted only
when PX4's `z_valid` is true, `epv` is at most `epv_max_m` and the EKF is not dead reckoning, and when one plane has
enough support, a tilt within `tilt_max_deg`, no more than `max_below_share` of the points below it (the ground is
the lowest surface) and a height within `height_tolerance_m` of PX4's camera height (z + 0.03 m). Two such planes
with comparable support at different heights are AMBIGUOUS. Otherwise the frame is refused (`FrameRefused`) with a
named reason: `estimator_vertical_fields_absent`, `estimator_z_invalid`, `estimator_epv_exceeds_limit`,
`estimator_dead_reckoning`, `ground_insufficient_points`, `ground_not_found`, `ground_insufficient_support`,
`ground_tilt_exceeds_tolerance`, `ground_not_lowest_surface`, `ground_height_disagrees_with_px4`,
`ground_inlier_spread_insufficient`, `ground_not_planar`, `ground_ambiguous`, `verified_camera_height_outside_band`.
The last three refuse wrong-surface fits (repair r1): any other horizontal plane found below the camera holding
`rival_min_share` of the points at a different height (a box top, a step, a platform); inliers covering less than
`min_half_share` of either image half or less than `min_range_span_m` of range; and a plane whose nearest and farthest
thirds or image halves, fitted alone, tilt more than `planarity_max_angle_deg` from it (a plane bridging the near
ground and a farther raised top). The floor filter, the virtual ceiling and the sky rule then use the verified
plane, never raw PX4 z. A valid return is floor when its height above the plane is below `margin_m` plus
z x (half a trained pixel + `floor_noise_margin_per_m`); stereo noise and the z-buffer's nearest-sample bias grow
with range.

**Sky rule (`apply_sky_rule`).** Only sensor-invalid pixels change; valid depth is never altered. An invalid pixel
becomes no-return if its ray never meets the verified plane and points above the horizon, its RGB is sky-like
(luma at least `luma_min`, local gradient against the neighbouring invalid pixels at most `gradient_max`) and it is
4-connected to the top of the image through such pixels or through pixels outside the source camera's field of view
(the top band a nose-down camera does not image; without it the open sky is a near wall on every bearing at 5-10 deg
nose-down). Pixels whose valid return the virtual ceiling removed are not connectors (repair r1): sky enclosed below
a removed canopy above 2.9 m stays sensor-invalid, a near phantom on every bearing it covers. Repair r2: only
unimaged pixels ABOVE the source camera's imaged region connect (above its top field-of-view edge, or in the
uncovered rectification border above the first covered row of a source column); uncovered side and bottom borders
never connect, so a rectification border no longer carries the sky flood down the image sides. An elevation margin
for these connectors was measured and not adopted (`REPAIR_r2.md`).

*Edge-enclosure veto (repair r1).* An invalid pixel lying between two near valid returns in its row (left and
right) or column (above and below) is never freed and blocks the sky flood: scanning through invalid pixels only, the
first non-invalid pixel on each side must be a valid return of the projection (floor returns excluded) within
`edge_veto_range_m` (5.0 m: the trained riskcap governor's 5.0 m release distance; worst-case stop from 3.54 m/s at
the trained 2.96 m/s^2 with 0.3 s delay is about 3.2 m), the two edges must subtend at most `edge_veto_span_deg`
(70 deg: a 2.4 m-wide face at 2 m spans 64.6 deg) and their ranges differ by at most `edge_veto_range_tolerance_m`
(1.0 m); the enclosed set then grows by `EDGE_VETO_DILATION_PX` = 2 invalid pixels, so a row whose edge sample is
unresolved (the field-of-view boundary of a nose-down camera) stays protected. The OAK-D's horizontal baseline usually
matches an object's vertical silhouette edges even where its face is bright and textureless, so such objects stay
near. Cost (synthetic, measured): sky seen between two near trunks
becomes a near phantom across the gap between them.

*One edge and oblique faces (repair r2).* In a ROW, a run of invalid pixels with a near valid return on only one side
is enclosed too when the other side is the image border or any stop that is not a near valid return (an unimaged or
uncovered pixel, a no-return beyond MaxZ, a valid return beyond `edge_veto_range_m`, a floor return): a face whose
other silhouette edge is out of view, unresolved or seen against far background. Columns keep the two-edge rule. Two
near edges whose ranges differ by more than the tolerance still enclose when the chord between their 3-D points is
seen at most `EDGE_VETO_MAX_INCIDENCE_DEG` = 70 deg off its normal (an oblique face; PROVISIONAL runtime constant).
**Cost (synthetic, measured; the safe direction):** one frame cannot tell such a face from sky beside a single near
object, so the sky in every row of any near valid return within 5.0 m that rises above the horizon (a tree trunk, a
post, a pole, a branch end) stays sensor-invalid and becomes near: one trunk inside that range, ahead or at the image
border, turns most bearings into a near wall, so the vehicle holds or turns away near trees; sky between trunks is near as
well. Objects beyond 5.0 m, low objects whose top stays below the camera, open sky, a tree line and an open field with
ranged far ground add nothing. Valid speckle inside that range in the sky has the same effect (measured with
synthetic speckle; the OAK-D's sky speckle is NOT_MEASURED). Numbers: `REPAIR_r2.md`.

*Far-ground rule (`far_ground_rule`, OFF by default, repair r1).* With it on, an invalid pixel whose ray meets the
plane only beyond MaxZ, or points below the horizon and never meets it, becomes no-return without any RGB test (the
band below the horizon is atan(h / MaxZ) wide: 2.9 deg at a 1.03 m camera). Depth alone cannot tell such far ground
from a dark, thin, sensor-invalid structure inside the vehicle's own altitude band (a rail, a barrier arm, a branch
just below the camera), which the rule deletes. **Enabling it requires both real OAK-D evidence that far ground drops
out to depth 0 (replay below) and the user's explicit acceptance that thin invalid obstacles inside the vehicle's
altitude band are then lost.** With it off, far ground reported as depth 0 stays sensor-invalid: in an open field
whose horizon is not occluded within 20 m it is a near band on every bearing (a phantom, the safe direction); far
ground that the OAK-D still ranges beyond 20 m is no-return as before.

Everything else stays sensor-invalid and becomes near under blob_near as before: dark branches, overhead objects,
near obstacles, textureless ground inside MaxZ, and invalid structures that reach at least about 3 trained rows below
the horizon (blob_near's 5 x 5 window at density 0.5). Repair r2 corrects the r1 wording ("at or just below the
horizon"): an object that is sensor-invalid with no valid edge at all, bright and smooth like the sky, is lost when it
lies wholly above the horizon OR reaches fewer than about 3 trained rows below it (a lower edge a few centimetres below
the camera), because its rows above the horizon are freed as sky and the rows below are too few to be near.
These classes cannot be told from sky in one frame; they are kept as `unittest.expectedFailure` tests that assert the
safe outcome (`tests/test_camera_runtime_outdoor_r1.py`, `FullyInvalidLimitations`, `RepairR2Residuals`), so the
limitation stays visible (expectedFailure). Keeping every invalid column run that crosses the horizon was measured and
not adopted: it keeps only some of them and makes the open sky a near wall (`REPAIR_r2.md`).

**PROVISIONAL thresholds.** Each is stored as `{"value": v, "status": "PROVISIONAL"}`, chosen on synthetic scenes
only, and must be calibrated on real OAK-D recordings (below):

| Block | Threshold | Value |
| --- | --- | --- |
| sky | luma_min | 0.5 |
| sky | gradient_max | 0.03 |
| sky | far_ground_rule | false |
| sky | edge_veto_range_m | 5.0 |
| sky | edge_veto_span_deg | 70.0 |
| sky | edge_veto_range_tolerance_m | 1.0 |
| ground_estimate | fit_max_range_m | 10.0 |
| ground_estimate | inlier_threshold_m | 0.06 |
| ground_estimate | min_inliers | 300 |
| ground_estimate | min_inlier_share | 0.25 |
| ground_estimate | tilt_max_deg | 3.0 |
| ground_estimate | height_tolerance_m | 0.3 |
| ground_estimate | epv_max_m | 0.5 |
| ground_estimate | ambiguity_support_ratio | 0.5 |
| ground_estimate | ambiguity_separation_m | 0.15 |
| ground_estimate | floor_noise_margin_per_m | 0.015 |
| ground_estimate | max_below_share | 0.02 |
| ground_estimate | rival_min_share | 0.1 |
| ground_estimate | min_half_share | 0.1 |
| ground_estimate | min_range_span_m | 1.0 |
| ground_estimate | planarity_max_angle_deg | 1.0 |
| ground_estimate | refusal_hold_max_s | 3.0 |

The ground fit's fixed runtime constants (`camera_policy_runtime.PROVISIONAL_RUNTIME_CONSTANTS`, e.g.
`GROUND_SEARCH_TILT_DEG` = 30) are PROVISIONAL too; `tilt_max_deg` is bounded to (0, 5] deg and
`height_tolerance_m` to (0, 0.5] m. `refusal_hold_max_s` (unit B, below) is bounded to [1.0, 10.0] s, both ends
inclusive.

**PX4 vertical validity reaches the runtime only as extra state fields.** The outdoor state carries `z_valid`, `epv`
[m] and `dead_reckoning` beside the five estimator fields (`camera_policy_runtime.split_state`; the indoor state is
unchanged and still refuses extra keys). Repair r1 refused every outdoor contract on the ROS node as UNCONFIGURED
with the named reason `outdoor_vertical_fields_not_forwarded`, because nothing on the ROS path carried these fields.

*Forwarding (unit B, 2026-11-18).* The PX4 gateway now forwards them inside its own private Odometry
(`/motar/state/odometry` by default), the same message whose pose the policy uses, so the policy node pairs nothing by
time. Encoding `motar-px4-vertical-v1` (`camera_ros2_adapter.VERTICAL_ENCODING`):

| Odometry slot | Content |
| --- | --- |
| `pose.covariance[0]` | tag -20261118.0 (a negative variance: never a valid covariance) |
| `pose.covariance[1]` | `z_valid`, 1.0 or 0.0 |
| `pose.covariance[2]` | `epv` in metres, finite, at least 0 |
| `pose.covariance[3]` | `dead_reckoning`, 1.0 or 0.0 |
| every other `pose.covariance` and `twist.covariance` slot | NaN |

The fields come from the PX4 `VehicleLocalPosition` sample that the gateway already pairs with the `VehicleOdometry`
sample to gate pose validity (same snapshot, capture skew at most 0.025 s). The gateway still revokes when `z_valid`
is false; it does not judge `epv` or `dead_reckoning` (the outdoor contract does). When a field is missing or
malformed it forwards none: every covariance stays NaN, exactly the message published before unit B. Besides the
unchanged `.msg` SHA-256 bindings, the gateway now checks the generated type of every field it reads on the
installed `px4_msgs` classes (`camera_ros2_px4.READ_FIELD_TYPES`: `epv` float, `dead_reckoning` boolean, ...), so a
mismatched build fails at construction. Its diagnostics report the encoding and the last forwarded fields.

*Supersedes (2026-11-18, review unitB r1 P2-2).* The earlier sentences "The private derived Odometry contains NaN
covariance entries ... The policy ignores covariance." above are kept unchanged only because a test pins that prefix;
they no longer hold. The gateway writes this encoding on every deployment, indoor included, so whenever the fields are
forwarded `pose.covariance[0]` reads -20261118.0 and slots 1-3 carry the fields. Only a policy node whose binding
declares the encoding (below) reads them. Any other reader of `/motar/state/odometry` must not treat
`pose.covariance` as a covariance.

The policy node reads the encoding only when its binding declares the gateway as the odometry source with the
optional binding key `"odometry_source": {"producer": "motar_px4_gateway", "vertical_encoding":
"motar-px4-vertical-v1"}` (`camera_ros2_adapter.gateway_odometry_source()`); any other value is refused. Without
that declaration an outdoor contract stays UNCONFIGURED with `outdoor_vertical_fields_not_forwarded` as before, and an
indoor session never reads the covariance. With it, each outdoor frame's state takes the fields of the Odometry
message used for its pose; for a pose interpolated between two Odometry samples both must carry the fields and they
combine conservatively (`z_valid` only if both, the larger `epv`, `dead_reckoning` if either). Absent or malformed
fields (another tag or version, filler that is not NaN, a flag that is not exactly 0 or 1, a negative or non-finite
`epv`) attach nothing, so the runtime refuses the frame with `estimator_vertical_fields_absent` (a zero hold); a
forwarded `z_valid` false, a large `epv` or dead reckoning give `estimator_z_invalid`,
`estimator_epv_exceeds_limit` or `estimator_dead_reckoning`. No default is ever substituted. The session's
diagnostics report the decode status of the last tick (`vertical_fields`). The CPU replays (NPZ `replay`,
`tools/oakd_outdoor_replay.py`) take the fields from their own inputs as before.

**Refused frames in the ROS session.** A `FrameRefused` frame does not advance the policy history. The session
answers it with an explicit zero-velocity, zero-yaw-rate hold proposal for that tick (same stamps and freshness
checks), reports `HOLD_FRAME_REFUSED` with the reason and the run length in the diagnostics, and starts a fresh
history on the next accepted frame. Re-entry hysteresis (repair r1): after a refusal the first
`REFUSAL_REENTRY_FRAMES` = 5 accepted frames (PROVISIONAL; 0.5 s, one trained history interval) still propose the
zero hold (`HOLD_REENTRY`, `reentry_hold_remaining` in the diagnostics) while the policy history refills, and the
runtime records the applied zero hold as its previous action; a refusal in between restarts the count. Repair r2:
with an outdoor contract the count restarts on every history reset, not only on a refusal: the session start or an
explicit reset, and an altitude-gated stretch (`WAITING_ALTITUDE`), so a nonzero command always follows at least
`REFUSAL_REENTRY_FRAMES` accepted frames of the current history; the NPZ `replay` applies the same hold (its line keeps
the policy's own command as `policy_*`) and `tools/oakd_outdoor_replay.py` marks such frames `reentry_hold`. Indoor
contracts never hold. Until unit B the hold had no time limit.

*Bounded hold (unit B, 2026-11-18).* A hold episode is the run of consecutive ticks whose proposal is the explicit zero
hold (`HOLD_FRAME_REFUSED` or `HOLD_REENTRY`) without an intervening policy command: a refusal inside it does not
restart it, a tick that proposes nothing (`WAITING_ALTITUDE`) neither extends nor ends it, and only a policy command or
an explicit session reset ends it. Its duration is measured on the held proposals' capture stamps (the declared ROS
clock): every proposal carries one, they strictly increase, each is checked within 0.15 s of the ROS clock, that clock
is checked against monotonic time and the 10 Hz period is enforced on it; the replays have the same stamps, and under
simulated time it is the vehicle's own time base. When the episode reaches `ground_estimate.refusal_hold_max_s`
(PROVISIONAL, 3.0 s by default, bounds [1.0, 10.0] s; not derived from a measurement), the session proposes nothing
more and latches the fault `refusal_hold_timeout` (status `FAULT`); there is no automatic re-entry, only the explicit
reset service (`~/reset_policy_session`, a fresh session) clears it. Held proposals therefore span less than the
limit. The gateway then revokes on its existing stale-command path (no fresh command within 0.15 s), so PX4's
offboard-loss failsafe (`COM_OF_LOSS_T`, then `COM_OBL_RC_ACT`) and the pilot take over; the last setpoint PX4 keeps
until then is the zero hold. No arm, land or mode command is sent. The node's
diagnostics report `hold_episode_s`, `hold_limit_s`, `hold_limit_status` and the `hold_timeout` record (reason,
episode length, held proposals, refused frames, last refusal). An exactly zero command from the policy itself is a
policy command and ends the episode. The NPZ `replay` and `tools/oakd_outdoor_replay.py` mark the frame at which the
session would latch (`refusal_hold_timeout: true`, `hold_episode_s` per held frame) and report it in their summary.

*Limits of the bounded hold (review unitB r1, 2026-11-18).*
- **Sparse commands defeat the bound (P2-1, review r1a).** A policy command ends the episode. Any refusal pattern that
  lets one policy command through before the hold reaches the limit therefore never latches: at most 25 consecutive
  refused ticks, then 5 re-entry holds and a command; the densest such pattern is every 7th tick. In the review probes
  this held the vehicle at zero for up to 97 % of ticks for 60 s with no fault (26 consecutive refusals did latch at
  3.0 s). Each such command follows `REFUSAL_REENTRY_FRAMES` accepted frames of fresh history, but the vehicle mostly
  holds and no fault alerts the pilot. A rate-based bound (share of held ticks in a window) is OPEN. Until it exists,
  the operator watches the diagnostics `state` (`HOLD_REENTRY` or `HOLD_FRAME_REFUSED`) and `hold_episode_s`, and takes
  over on a persistent hold pattern. (`refusal` and `consecutive_refusals` reset on every accepted frame, so they stay
  near null and 0 in a sparse pattern.)
- **Gated time counts.** "Neither extends nor ends" above means a `WAITING_ALTITUDE` tick does not end the episode.
  The duration runs from the episode's first held capture stamp, so time spent gated still counts. If an episode is
  open when the gated stretch begins (the last proposal was a zero hold, for example refusals during the descent), a
  re-climb in the same session latches at its first hold; reset the session before re-arming. The latch can also fire
  before the pilot selects OFFBOARD (for example, ground unverifiable for the limit right after take-off); the
  operator then resets the session before the handover.
- **Clock.** The checks above catch jumps, pauses and a wrong period, but not a ROS clock that runs uniformly slow.
  The review estimates that the hold can then stretch to about 1.33-1.5 times the limit in wall time.
- **Replay stamps.** The NPZ `replay` now stops on a duplicate capture stamp between consecutive held frames, where it
  used to finish. Recordings must carry strictly increasing stamps.

Every other input error still faults the session. Before the zero-hold change of 2026-11-13 a refusal
faulted the session, the proposal stream stopped, the gateway revoked within about 0.15 s, and PX4 kept the last
nonzero velocity setpoint until its offboard-loss timeout (`COM_OF_LOSS_T`, 1.0 s by default in v1.14.3) before
`COM_OBL_RC_ACT`. Any other session fault, or a stopped node, still leaves that window: shorten `COM_OF_LOSS_T` for
policy flights, and treat a gateway brake-on-loss as an open item. The gateway geofence is unchanged.

**Limitations (outdoor GO is not given).** Wires 1-3 px thick reported invalid are not detected, nor are invalid
structures thinner than about 3 trained rows (a 0.10 m rail at 5 m; blob_near needs a dense blob). A sensor-invalid,
bright, smooth object with no valid edge lying wholly above the horizon, or reaching fewer than about 3 trained rows
below it, is lost when it connects to the top of the image (directly, through low-contrast sky, or through the
unimaged band of a nose-down camera). A bright face whose only valid edge lies beyond 5.0 m ray range, or is narrower
than about one trained pixel on the tilted path, is lost too. Textured bright
clouds, sky between near trunks, sky in the rows of any near valid return (repair r2), sky below a removed canopy,
invalid canopies above the virtual ceiling, invalid far ground in an open field (also a thin depth-0 band just below
the horizon) and a saturated sun blob below the horizon stay near (phantom obstacles). A wide horizontal
surface that hides all ground within the fit range is taken as the ground when its top lies within the PX4 height
tolerance; low wide boxes beside visible ground are now refused. The site must be flat (one
plane, slope within the tilt tolerance); with more stereo noise than modelled, flat ground is refused more often
(0.5 px disparity noise at a 2.0 m camera: refused, a zero hold). The thresholds and the stereo noise model are
synthetic. Measured results:
`/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/OUTDOOR_r1.md` and, after the independent reviews, `REPAIR_r1.md`
and `REPAIR_r2.md` in the same folder.

### OAK-D Pro (the user's camera, confirmed 2026-11-15)

Luxonis documentation, accessed 2026-11-15: [OAK-D Pro product page](https://docs.luxonis.com/hardware/products/OAK-D%20Pro),
[OAK-D Pro datasheet](https://github.com/luxonis/depthai-hardware/blob/master/DM9098_OAK-D-Pro/Datasheet/OAK-D-Pro_Datasheet.pdf),
[DepthAI v2.29.0 `DeviceBase.hpp`](https://raw.githubusercontent.com/luxonis/depthai-core/v2.29.0/include/depthai/device/DeviceBase.hpp),
[depthai-python v2.29.0.0 `DeviceBindings.cpp`](https://raw.githubusercontent.com/luxonis/depthai-python/v2.29.0.0/src/DeviceBindings.cpp).
None of it is measured on the user's unit; the variant (auto or fixed focus) is NOT_MEASURED.

| Item | Documented value |
| --- | --- |
| Stereo pair | OV9282, global shutter, 1280 x 800, DFOV / HFOV / VFOV 89.5 / 80 / 55 deg, fixed focus |
| RGB, auto-focus variant | IMX378, 4056 x 3040, 78 / 66 / 54 deg |
| RGB, fixed-focus variant | IMX378, 4056 x 3040, 82 / 69 / 55 deg |
| Baseline | 7.5 cm |
| IR dot projector / IR flood LED | 940 nm each, up to 1 W each; the stereo pair has 940 nm notch filters, the RGB camera does not see IR |
| IMU | BNO086 |
| Depth | ideal range about 0.8-12 m; depth 0 means invalid; disparity search 0-95 px (extended 0-190 px) |

DepthAI 2.29 sets the emitters on the device: `setIrLaserDotProjectorIntensity(intensity, mask=-1)` and
`setIrFloodLightIntensity(intensity, mask=-1)`, intensity 0..1, 0 or negative turns the emitter off ("The emitter is
turned off by default"); the mA setters `setIrLaserDotProjectorBrightness` / `setIrFloodLightBrightness` are
deprecated; `getIrDrivers()` lists the LED driver. The runtime and `tools/oakd_camera.build_pipeline` never call them,
so the projector stays off by default; this unit does not change that.

Consequences for this runtime (PROVISIONAL until measured on the unit): the `oakd_like` test preset (1014 x 760,
69 deg HFOV) matches the fixed-focus variant. The auto-focus variant's 66 deg HFOV equals the trained HFOV and, on a
4:3 sensor, gives about 51.9 deg VFOV (the page lists 54 deg), below the trained 54 deg: the level path then refuses
every frame (`source camera does not cover the trained FOV`) and `oakd_camera.Rectifier.require_training_fov` refuses
the intrinsics (fail closed; `OakdProPresets`). The fixed-focus variant leaves about 8 source pixels above and below
the trained rows, so a wider rectification border uncovers the trained image's top rows (top-band connectors).
Whether far ground beyond 20 m, the sky and the stereo search band at one image side read as depth 0 in the
RGB-aligned depth is NOT_MEASURED (capture items below).

### Recording the user's OAK-D outdoors for calibration (no extra sensors)

`tools/oakd_outdoor_replay.py` replays recorded frames through the outdoor profile without DepthAI, a checkpoint or
ROS, and writes per-frame refusal, ground, sky and near statistics plus a summary. Its docstring defines the folder
format (`recording.json`, `frames/NNNNNN.npz` with RGB, depth in mm with 0 invalid, gravity and time).

1. Mount the OAK-D Pro on a tripod with the lens on the tripod axis. Measure the height of the RGB optical centre
   above the ground with a tape (1.0 m and 1.5 m) and write it to `camera_height_m` with its reading uncertainty.
   Record in `device` the model and variant (OAK-D Pro, auto or fixed focus RGB), the serial number (MX ID) and the
   DepthAI version, and keep the device calibration (`readCalibration`) beside the recording.
2. Capture with the existing pipeline: `tools/oakd_camera.build_pipeline` (12 MP ISP 1/4 RGB, 800P stereo,
   depth aligned to RGB, measured device calibration) and `Rectifier.apply`, saving the rectified RGB, the
   rectified depth in mm and the coverage. That pipeline has no IMU node: add a DepthAI `IMU` node (accelerometer,
   about 100 Hz), average it over each still frame, rotate it into the RGB optical frame with the device's
   IMU-to-camera extrinsics from the device calibration, and store the up vector (`up_optical`). Without an
   extrinsic, record `roll_pitch_deg` from the IMU's own orientation and state how it was aligned.
3. For each height, record pitches 0 and plus/minus 10 deg (positive = nose down), 20-50 frames each, at a flat,
   short-grass or paved site: open field to the horizon; a tree line; a tree trunk at 0.3 m and 1.5 m; a low branch
   above camera height; overhead canopy or a roof edge; a pole or post; a wire against the sky if available; clear,
   partly cloudy and low-sun skies; a gentle slope; a low wide object (box or bench). Note distances in `scene`.
4. Build an outdoor contract and run
   `python -B tools/oakd_outdoor_replay.py --contract outdoor_contract.json --recording <folder> --output <new dir>`.
   Report the height error against the tape, the refusal reasons, the sky and far-ground pixel counts and the near
   columns per scene before changing any PROVISIONAL threshold. Recordings are raw data: keep them outside Git and
   bind them by path and SHA-256.
5. OAK-D Pro IR projector (repair r2): record every scene twice, with the IR dot projector off (the boot default) and
   on (`setIrLaserDotProjectorIntensity`, the value written in `scene`), flood LED off. Sunlight usually washes the
   940 nm dots out; in shade and at short range they may add texture. Note sun or shade per scene.
6. Capture items for the open questions (each with the projector off and on):
   - rectification border: the `coverage` mask and calibrated intrinsics of the unit; the uncovered border width at the
     top, sides and bottom of the 1014 x 760 image, and whether `Rectifier.require_training_fov` passes;
   - stereo search band: the width of a permanent depth-0 band at one side of the RGB-aligned depth, in both stereo
     modes (`extended`, `subpixel`);
   - far ground: an open field to the horizon at 1.0 and 1.5 m; the share of depth-0 pixels in the bands 0-1, 1-2 and
     2-3 deg below the horizon, and the farthest valid ground range;
   - bright faces: a white and a light-grey board at least 1 m wide at 1.5, 2, 3 and 4 m, against open sky and against
     a tree line, frontal with both vertical edges in view, with one edge out of view, and turned 30-60 deg; whether
     the vertical silhouette edge columns are valid and at what depth, and whether the face itself is depth 0;
   - sky: clear, partly cloudy, overcast, sun in view and low sun near the horizon; the count and ranges of valid
     speckle pixels in open sky (one near speckle row makes the sky near under the one-edge rule);
   - thin structures: wires and branches 1-5 m away, 1-3 source pixels thick, against sky;
   - flat ground noise: the depth spread on flat short grass and on pavement at 2-10 m (for the ground-fit margins).
