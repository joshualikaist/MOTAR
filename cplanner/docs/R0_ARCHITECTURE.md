# P0-R Gate R0: architecture design (written before any P0-R code)

Date 2026-10-29. P0 v7 stays **FAIL** (`/home/fair/motar_tmp/p0_v7_independent_20261029/P0_V7_REVIEW_STATUS.md`).
This is a redesign, not a v8 patch. CPU only; no production, Frozen F, raw, checkpoint or earlier-evidence change.

## 1. Why v1-v7 failed, and what is structurally different

| Failure family | v-history | Root cause | P0-R structure |
| --- | --- | --- | --- |
| Caller code inside a check | v5 clock object, v6 `str`-subclass dict keys, v7 metaclass `__eq__` in `type(x) in (A, B)` | The boundary accepted **Python objects** (dataclasses, dicts, tuples, enums) and read them through protocols the caller can override. Every repair closed one protocol and left another. | The per-step boundary accepts **only exact `numpy.ndarray`s** of an exact native numeric dtype class and an exact shape. They are checked with identity tests (`type(a) is np.ndarray`, `type(a.dtype) is <float64 dtype class>`) and C-level attributes, then copied once into core-owned read-only arrays. No signed records, no HMAC tags, no caller-held objects that the core later trusts. Configuration (manifest, sensor spec, bar catalog, plan radius) comes from an internal registry chosen by name at construction. |
| Camera lifts a LiDAR veto | v6 window association, v7 range gate (F1: noisy estimate lifts a non-target up to R + 6 sigma) | The certificate had a code path in which camera evidence **removed** a surface. Any association rule with an uncertain estimate admits a wrong removal. | **No such code path exists.** The certificate function has no camera parameter. Every validated HIT, target or not, is a surface veto. Camera output goes to a separate label layer that the certificate and the occupancy map never read. |
| Silent loss of evidence | v5 R-1 (rejected returns vanished), MOTAR NaN/miss -> 12 m | Missing/invalid beams were indistinguishable from NO_HIT. | Explicit per-beam status array; only declared `HIT`/`NO_HIT` with consistent ranges validate; anything else makes the packet incomplete, and an incomplete packet never certifies (fail-closed, R-1 kept). Range alone never implies HIT/NO_HIT. |
| Stale state used | v5 R-5 | Ledger state not tied to the certificate's packet. | The certificate uses only the core's **current** packet. A refused step clears it (`NO_CURRENT_PACKET`). Steps must strictly increase. |

## 2. Minimal numeric data format

Per step (all four are required; nothing else is read):

| Array | dtype | shape | Content |
| --- | --- | --- | --- |
| `ranges` | float64 | (R, A) of the manifest | delivered range in metres |
| `status` | int8 | (R, A) | declared per-beam status code (below) |
| `header_i` | int64 | (5,) | env_id, episode_id, acquisition_step, health_code, noise_applied |
| `header_f` | float64 | (5,) | timestamp, sensor x, y, z (world), yaw (world, rad) |

Optional per step (labels / liveness only, see §4): `camera` float64 (14,), `track` float64 (6,).
Per certificate query: `paths` float64 (K, 2, 2), 1 <= K <= 64, world-frame straight segments.

Status codes: 0 NOT_DELIVERED (MISSING), 1 HIT, 2 NO_HIT, 3 SENSOR_ERROR, 4 DROPOUT, 5 NEAR_CLIPPED, 6 DISTANCE_ONLY.
Validation per beam: HIT needs a finite range in [min_range, max_range]; NO_HIT needs range == no_hit_fill exactly;
3/4/unknown codes and non-finite ranges -> INVALID; 5/6 and inconsistent HIT/NO_HIT -> AMBIGUOUS. A packet with
health_code UNKNOWN, synthetic health under a runtime config, or noise_applied (no qualified error bound, R-6) makes
every beam AMBIGUOUS.

## 3. Sensor packet contract

- **Manifests (internal registry).**
  - `P0R-72x1`: the P0 synthetic planar ring, azimuth 5 deg x j from 0 deg (identical to v3-v7 beams).
  - `MOTAR-72x4`: rows elevation 20 - 10 i deg (i = 0..3: +20, +10, 0, -10), columns azimuth 180 - 5 j deg
    (j = 0..71: +180 ... -175, decreasing), id = 72 i + j. Source: `W/aerial_gym/sensors/warp/warp_lidar.py:76-84`
    with `navrl_lidar_config.py:60-61` (min -175, max 180); flattening `navrl_task.py:6017`. Directions are fixed in
    the manifest; the caller never supplies a direction.
- **Frame.** Yaw-level sensor frame at the declared sensor position; world direction = Rz(yaw) d (MOTAR attaches the
  LiDAR yaw-only, `warp_sensor.py:184-212`). Pose is a declaration (simulation odometry); pose error is out of scope.
- **Time.** `TIME_BASIS = PACKET_ACQUISITION_STEP`. There is no caller `now`. A certificate is valid for the step of
  the packet it was computed from (`valid_for_step`). The header timestamp must equal step x dt (dt = 0.1 s,
  `DECLARED_SIM_SCHEDULE`); steps must strictly increase per core (replay/out-of-order refused).
- **Completeness (R-1 kept).** Every manifest beam must validate as HIT or NO_HIT, else the packet is INCOMPLETE:
  it still contributes its validated HITs as occupancy to the map, adds no free evidence, withdraws visibility along
  every non-validated beam, and can never certify.
- **Free evidence.** Along a validated beam, free interval [f0, f1]: f0 = 0 if the config declares nearest-hit
  assurance else min_range (P0-C blind-zone rule kept); f1 = range - 1e-7 for HIT, max_range for NO_HIT;
  range_error_bound must be 0 (R-6 kept: a positive bound refuses). For non-planar layouts only the part of the ray
  inside the catalog bars' vertical extent [0, 3] m is 2-D free evidence (a full-height bar in that 2-D spot would
  have been hit there). MOTAR has no floor mesh (`navrl_bars_env.py:60`), so the -10 deg row reads 12 m beyond
  about 5.7 m; that part is below z = 0 and is not evidence.
- **Format validation is not sensor-health validation.** Health, statuses, ranges and pose are declarations. The
  evidence label is `RUNTIME_DECLARED` / `SYNTHETIC_ASSUMED_HEALTH`, `health_basis = DECLARED_NOT_AUTHENTICATED`.

## 4. Camera vs LiDAR roles

| Use | Camera / tracker | LiDAR |
| --- | --- | --- |
| Target position, velocity, prediction, occlusion, reacquisition, goal | yes (label layer, P1/G1) | supporting |
| Delete a LiDAR return / lift a surface veto | **never** (no code path) | - |
| Delete static occupancy, turn UNKNOWN into FREE, certify unobserved space | **never** | only validated free rays create free evidence |
| Static-occupancy confirmation (N/T/D rule with a fresh track, kept from v7) | the track may only make a cell **more** conservative (permanent veto) | hits |

Guarantee to be tested in R2: certificate outputs are **byte-identical with and without camera input**, and with a
track can only lose certificates relative to without (monotone).

## 5. Static Bar Safety Certificate: scope

The certificate is `STATIC_BAR_CERTIFIED` for one world-frame segment and one packet iff all of:
1. the packet is complete (every beam validated);
2. no validated HIT (any object, target included) is within R = 0.45 m of the segment;
3. `_coverage_B` (verbatim from v3-v7, AST-identical) excludes every centre cell of every catalog bar shape that
   could come within R of the segment, using only this packet's free rays;
4. the occupancy map has no veto cell (OBSERVED_HIT, MOVING_TARGET_CANDIDATE, STATIC_OCCUPANCY, CONFLICT) within R
   (plus half a cell diagonal) of the segment;
5. if the config declares an arena, the swept disc stays inside it.

Assumptions under which 3 is a proof (sufficient, not necessary): bars are static, upright, full-height (0-3 m),
axis-aligned, from the declared catalog (MOTAR `bars_h3`: 40 boxes, rpy 0, placement yaw 0,
`env_object_config.py:818-857`); free rays are what the sensor declares; pose is exact.
Every output carries scope fields, never a single SAFE flag: `dynamic_objects`, `non_catalog_objects`,
`sensor_provenance`, `pose_error`, `tracking_error`, `terminal_approach`, `timing` (all UNVERIFIED/UNCERTIFIED/
DECLARED), plus `arena_boundary`.

## 6. UNKNOWN and non-catalog objects

- UNKNOWN in the map never vetoes and never grants. The certificate never states FREE; it states "no catalog bar"
  for the segment.
- A non-catalog object that a ray hits is a surface veto (rule 2) and a map veto (rule 4). One between beams or
  smaller than the catalog is **not covered** (`non_catalog_objects: UNVERIFIED`); R2 reports such certified cases
  as OUT_OF_SCOPE, never as safe.
- Map states: VALIDATED_FREE, OBSERVED_HIT, STATIC_OCCUPANCY, MOVING_TARGET_CANDIDATE, UNKNOWN, CONFLICT. Only free
  rays of complete packets add free evidence; an occupancy trace resolves only after K_FREE = 2 distinct later
  packets with free evidence through the cell; time alone never makes anything FREE; an expired target label
  becomes OBSERVED_HIT, never FREE; STATIC never resolves (free after it -> CONFLICT, permanent). The map can only
  add vetoes to the certificate, so a map defect can cost liveness, not certificate soundness for catalog bars.

## 7. Terminal approach is a separate problem

With R = 0.45 and the target sphere 0.20, keeping the certificate requires a centre distance >= 0.65 m, but capture
needs < 0.5 m. P0-R therefore cannot certify capture; such segments are refused and classified
`TERMINAL_APPROACH_UNCERTIFIED`. The terminal mode is specified (not implemented) in `TERMINAL_INTERCEPTION_SPEC.md`.

## 8. Failure behaviour

- Malformed arrays, wrong dtype/shape/subclass, ids out of range, step not increasing, timestamp != step x dt,
  non-finite pose -> the step is refused, the current packet is cleared, nothing in the map changes.
- Any exception inside step/certify is caught, logged, and returns a refusal (never raises on argument values;
  R-4 kept). Constructor misuse raises (programming error).
- Coverage node budget exceeded -> `NODE_BUDGET_UNKNOWN`; an optional wall-clock deadline per query refuses the
  remaining candidates with `DEADLINE_UNKNOWN`. A refusal is never a command: the safe fallback command (brake/hover)
  is the planner's responsibility (G1/G3), not provided here.

## 9. Interface needed for MOTAR (P1, not built here)

`ranges` = `_lidar_distance_m()` reshaped (4, 72) **before** its nan/inf->12 conversion is not available today;
the status channel does not exist (miss becomes 12 at `warp_sensor.py:236`; NaN/inf become 1.0 -> 12 m at
`navrl_task.py:6015`; near < 0.2 m becomes -12 then 0). Until V1 qualifies a derived HIT bit (or a production flag
exists), a MOTAR adapter must send `DISTANCE_ONLY` (AMBIGUOUS) and nothing certifies. The header comes from
`sim_steps`, env/episode ids, and the robot pose (x, y, z, yaw).

## 10. Not implemented in P0-R

P1 accumulated world map with real 72x4 data, the MOTAR adapter and V1 qualification; candidate generation and
optimisation (G1); dynamics/tracking-error model (G2); terminal mode, reacquisition, real-time budget (G3); GPU
shadow (G4); closed-loop comparison (G5); Track B.

## Safety self-check before coding

No contradiction found between the safety claim (catalog-bar freedom of a segment under the assumptions of §5) and
the rules above: free evidence only shrinks, every return vetoes, the map only vetoes, the camera has no path. The
one known conflict is functional, not a soundness one: capture < 0.5 m is incompatible with treating the target as
an obstacle at R = 0.45 (§7), so the certificate cannot cover capture by construction.
