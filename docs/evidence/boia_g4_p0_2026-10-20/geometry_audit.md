# BOIA P0 geometry and visibility source audit

Base `13d5f195399bf1a7f1cdf9ddeccd3338d8e34c0f`. CPU synthetic fixtures only. The corrected implementation remains a planar evaluator model; this is not qualification of physical free space, camera realism, 3D collision safety or Recovery performance.

## Reproduced faults and retained blockers

| ID | Base location | Independent counterexample | Disposition |
| --- | --- | --- | --- |
| A01 (P0) | `geometry.py:21-29` | Sigma=0,rT=.15 gives sqrt(9.21)*rT=.45522m deletion radius; hit5.8m against belief6m is deleted. Cov=.01 deletes .53m foreground; large covariance, false association and mismatched camera depth are also reproduced. | FIXED_AS_FAIL_CLOSED_ABSTENTION: ABSTAIN from identity and retain all valid first hits; API retained. |
| B01 (P0) | `geometry.py:47-54` | 20deg slant5m -> horizontal4.698463103929543m; old radius5m falsely admits q=4.85m. Projecting only the selected row minimum is also wrong when 0deg4.9 competes with20deg5. | FIXED_WITHIN_DECLARED_PLANAR_CONTRACT: Project every row, then row minimum and horizontal margin. Default declared4row contract; explicit elevations for nonstandard rows. |
| B02 (P0) | `visibility.py:60-62,111` | ego(0,0),camera(.1,0),target(4,2),obstacle(2,.6),rOcc=.35: body-line distance .3577708764 (clear), optical-line .3331111527 (blocked), target in FOV. | FIXED_WITHIN_YAW_ONLY_XY_MODEL: Use yaw-rotated optical origin for both LOS paths; original LiDAR context stays at original scan pose. |
| B03 (MODEL_BLOCKER) | `geometry.py:47-54,57-111; visibility.py:17-27` | A radius.2m obstacle at5m and bearing2.5deg misses neighboring0/5deg rays: nearest-ray distance5*sin(2.5deg)=.2180969368>.2. All-no-return scan can nevertheless label its centre free. | BLOCKED_FOR_PHYSICAL_FREE_SPACE_CERTIFICATION: No unsupported ray filling or physical-free claim; retained explicit model limitation. |
| B04 (MODEL_BLOCKER) | `geometry.py:47-54; visibility.py:17-27,38-64` | Target centre6m, raw first hit5.8m, margin.1m -> original sector endpoint5.7m and visible target hypothesis beyond it. Even excluding a point would not create measured free space behind the hit. | BLOCKED_UNRESOLVED: Preserved fail-closed BLOCKED. A01 retains true target too; no target-like return is filled to far. |
| B05 (P1_INPUT_CONTRACT) | `geometry.py:76-82,85-111` | An unqueried inf/negative/above-far sector or NaN yaw/z-only pose may still return true; nonpositive/nonfinite far can admit origin. | FIXED_FAIL_CLOSED: Invalidate whole env context row; sanitize only local arithmetic and AND validity mask on every answer; no source data writes or inf clipping. |

A01 deliberately retains the actual target as an obstacle too. This is ABSTAIN, not an identity classifier. It preserves measured first hits but does not solve B04. The 6m/5.8m case stays BLOCKED. No first-hit ray is filled to far and no evaluator-only identity or GT enters the functions.

B02 changes only the LOS start in `forecast` and `bpod_analytic` to `p_ego,XY + R_yaw*camera_offset,XY`. The existing FOV transform still subtracts the camera offset once. The known-free region remains in the original LiDAR scan frame.

B03 is a remaining physical-model blocker. Exact segment partitioning proves membership only in the interpolated radial-sector set, whose continuous physical coverage was never observed. The sampled-ray counterexample is not repaired by numerical tolerances.

## Function-by-function contracts

All 14 base functions and both added helpers are covered below. Shapes use N environments, B azimuth bins, V elevation rows, P points, K candidates, M target paths, H supplied time indices, and *Q arbitrary query dimensions. None of these functions advances dt. No physical sensor timing or latency has been calibrated by this audit.

### `geometry.py::lidar_points_xy` (base 9–18; current 13–22)

Input: rho (N,V*B) slant metres; azimuth (B), elevation (V), yaw (N) radians; pos (N,3) world metres. Output: points (N,V*B,2) world XY metres and validity (N,V*B) bool.

Assumptions/limits: Yaw-only LiDAR orientation; horizontal distance rho*cos(elevation), finite positive range strictly below .995*far is a measured hit. No surface extent or beam footprint reconstruction.

Evidence: `HorizontalFreeCounterexamples.test_invalid_and_nearzero_ranges_cannot_expand_free_space`. Qualification remains LIMITED_VALIDITY for the declared model.

### `geometry.py::exclude_target` (base 21–29; current 25–38)

Input: points (N,P,2) world XY m; valid (N,P) bool; belief mean (N,2) m and covariance (N,2,2) m^2; target radius m and chi2 dimensionless. Output: independent cloned validity (N,P) bool.

Assumptions/limits: ABSTAIN: beliefs cannot prove identity. All previously valid target and obstacle hits are retained. No semantic/GT input; target self-blocking remains.

Evidence: `ExclusionCounterexamples: six object-role cases plus mask/order/no-mutation`. Qualification remains LIMITED_VALIDITY for the declared model.

### `geometry.py::seg_point_dist` (base 32–44; current 41–53)

Input: broadcast segment endpoints (N,*Q,2) world XY m; measured points (N,P,2) world XY m. Output: nonnegative distances (N,*Q,P) m.

Assumptions/limits: Euclidean closed-segment projection with clamped [0,1] parameter; zero-length segment reduces to point distance. Assumes finite valid geometry supplied by caller.

Evidence: `VisibilityOriginsAndModelLimits.test_segment_distance_zero_length_and_partition_independent_examples; independent Python scalar projection`. Qualification remains LIMITED_VALIDITY for the declared model.

### `geometry.py::known_free` (base 47–54; current 56–83)

Input: rho (N,V*B) slant m; azimuth (B) rad; pos (N,3) m, yaw (N) rad; far and horizontal margin m; optional elevations (V) rad. Output: scan-relative radii (N,B), horizontal m.

Assumptions/limits: Default six-argument contract is explicitly four elevations [20,10,0,-10] deg. Other layouts require elevations. Project rows before row minimum, subtract horizontal margin, then adjacent-bin minimum. Invalid/nonpositive evidence never expands a ray; no target-like far fill. Sparse-sector and 2D assumptions remain.

Evidence: `HorizontalFreeCounterexamples: 0,+/-10,+/-20 deg; before-min projection; undeclared layouts; invalid/near-zero returns`. Qualification remains LIMITED_VALIDITY for the declared model.

### `geometry.py::_radii` (base 57–73; current 86–102)

Input: relative XY queries (N,*Q,2) m; yaw (N) rad; sector radii (N,B) m; angles (B) rad. Output: assigned radii (N,*Q) m.

Assumptions/limits: Requires >=3 uniformly decreasing full-circle rays; nearest-ray sectors use smaller adjacent radius exactly at numerical boundary (8 machine eps). Sector interpolation is a model, not physical beam coverage.

Evidence: `IndependentHelperChecks.test_sector_helper_uses_minimum_on_boundary`. Qualification remains LIMITED_VALIDITY for the declared model.

### `geometry.py::_safe_context` (new helper; current 105–121)

Input: pos (N,3) m; yaw (N) rad; radii (N,B) m; far scalar m. Output: valid env mask (N) and locally sanitized pose/yaw/radii with same shapes.

Assumptions/limits: Whole env invalid on any nonfinite position/yaw/radius, negative/above-far radius, or nonfinite/nonpositive far. Local zero values must always be combined with valid mask. No scan mutation.

Evidence: `ContextInputCounterexamples: point and segment checked separately with valid second env`. Qualification remains LIMITED_VALIDITY for the declared model.

### `geometry.py::point_in_known_free` (base 76–82; current 124–131)

Input: queries (N,*Q,2) world XY m and original scan pos/yaw/radii/angles/far. Output: bool membership (N,*Q).

Assumptions/limits: Original LiDAR origin, per-env input guard, finite query, radius bound. Valid finite contexts preserve nearest-sector behavior. Does not certify unobserved inter-ray space or altitude clearance.

Evidence: `HorizontalFreeCounterexamples; ContextInputCounterexamples; sparse scan limitation fixture`. Qualification remains LIMITED_VALIDITY for the declared model.

### `geometry.py::segment_in_known_free` (base 85–111; current 134–161)

Input: broadcast endpoints (N,*Q,2) world XY m; original FreeContext. Output: bool segment membership (N,*Q).

Assumptions/limits: Partitions segment at sector-boundary intersections; radial convexity makes endpoint max sufficient within each sector. Exact for represented 2D radial-sector model only. Zero-length segments and invalid context guard included; no physical 3D guarantee.

Evidence: `Scalar segment examples; ContextInputCounterexamples; retained 6/5.8 negative`. Qualification remains LIMITED_VALIDITY for the declared model.

### `visibility.py::fov_mask` (base 9–14; current 9–14)

Input: camera-frame vectors (...,3) m; hfov/vfov rad; radial max range m. Output: bool (...), forward x and finite vectors.

Assumptions/limits: Yaw-only pinhole centre FOV and Euclidean radial range. Exact floating boundaries have no numerical tolerance; silhouette/partial visibility and exposure effects are not represented.

Evidence: `IndependentHelperChecks.test_fov_and_camera_transform_use_declared_yaw_frame; interior/exterior margin .01m`. Qualification remains LIMITED_VALIDITY for the declared model.

### `visibility.py::los_clear` (base 17–27; current 17–27)

Input: explicit start/end (N,*Q,2) world XY m; obstacle points (N,P,2) m; valid mask; disc radius m; optional original FreeContext. Output: bool LOS (N,*Q).

Assumptions/limits: Point-disc segment clearance and optionally original scan-sector coverage. Caller must provide optical origin for camera LOS; no height, silhouette, material/transparency, rolling shutter or new-depth synthesis.

Evidence: `Camera-origin scalar oracle and retained 6/5.8 negative; geometry scalar-distance check`. Qualification remains LIMITED_VALIDITY for the declared model.

### `visibility.py::_to_camera` (base 30–35; current 30–35)

Input: world-relative vectors (N,*Q,3) m; yaw (N,*prefix) rad; declared yaw-frame camera offset m. Output: camera-relative vectors with same shape, m.

Assumptions/limits: Inverse yaw rotation then subtract camera offset exactly once. Roll/pitch and full SE(3) calibration are not modeled.

Evidence: `IndependentHelperChecks scalar +90deg rotation/offset; rotated LOS fixture`. Qualification remains LIMITED_VALIDITY for the declared model.

### `visibility.py::_camera_origin_xy` (new helper; current 38–43)

Input: ego position (N,*Q,3) world m; yaw (N,*Q) rad; camera offset in vehicle yaw frame m. Output: optical origin (N,*Q,2) world XY m.

Assumptions/limits: Added minimal B02 fix: p_ego,XY + R_yaw*camera_offset,XY. Vertical offset is not a planar LOS certificate. Does not alter original LiDAR free-set origin.

Evidence: `Scalar (0.1,0) optical origin and +90deg rotated counterexample`. Qualification remains LIMITED_VALIDITY for the declared model.

### `visibility.py::forecast` (base 38–64; current 46–73)

Input: ego rollout (N,K,H,3) world m; yaw (N,K,H) rad; target sample paths (N,M,H,6) [m,m/s]; obstacle points, free context, camera and explicit priors. Output: P_det (N,K,M,H), dimensionless.

Assumptions/limits: Uses original static point map, optical-origin LOS, yaw-only camera and projected-area gate. Oracle means this declared geometry model, not simulator GT. Stress p_miss/beta/n0 are explicit uncalibrated priors. Uses params r_T/min_pixels; caller must match camera calibration. No online GT.

Evidence: `Base false-clear versus independent scalar oracle; corrected optical origin and no-double-offset tests; retained negative`. Qualification remains LIMITED_VALIDITY for the declared model.

### `visibility.py::p_any` (base 67–70; current 76–79)

Input: joint sampled detection probabilities (N,K,M,H). Output: per-candidate probabilities (N,K).

Assumptions/limits: First product/union within each sampled trajectory, then average over M. Within-path Bernoulli draw independence is a stress-model assumption; geometry 0/1 case is deterministic. Not union of separately averaged marginals.

Evidence: `IndependentHelperChecks.test_union_is_computed_per_joint_path_before_averaging, hand result .72 versus .66`. Qualification remains LIMITED_VALIDITY for the declared model.

### `visibility.py::_normal_clearance` (base 73–80; current 82–89)

Input: signed margin g m; gradient dimensionless; symmetric PSD covariance m^2; optional strict boolean boundary. Output: linearized Gaussian clearance probability with g shape.

Assumptions/limits: Gradient covariance contraction and scalar normal CDF. Valid PSD covariance is caller contract; numerical negative variance clamped to zero. Deterministic zero-variance boundary uses strict forward gate where requested.

Evidence: `IndependentHelperChecks scalar erf oracle at variance4, g=-1/0/1; strict zero-variance case`. Qualification remains LIMITED_VALIDITY for the declared model.

### `visibility.py::bpod_analytic` (base 83–122; current 92–132)

Input: ego (N,*Q,3) m; yaw (N,*Q) rad; target mean (N,3) m; covariance (N,3,3) m^2; measured points/valid/radius; camera. Output: dimensionless probability (N,*Q).

Assumptions/limits: Fixed-ego, active-margin FOV/size/range linearization and independent point-disc clearance products. B02 corrected optical-origin mean segment and its derivative. Unknown-space probability omitted by design; forecast comparison requires declared model alignment. Uses cam target_radius/min_pixels (caller must match params).

Evidence: `Base false-clear versus scalar geometry; corrected zero-covariance optical-origin result; normal-CDF helper oracle`. Qualification remains LIMITED_VALIDITY for the declared model.

## Tests and provenance

Initial unchanged-base run: 17 methods; [{'test': 'test_camera_lidar_depth_mismatch_is_conservatively_preserved (test_boia_p0_geometry.ExclusionCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 67, in test_camera_lidar_depth_mismatch_is_conservatively_preserved\n    self.assertTrue(bool(retained((4., 0.), mean=(6., 0.), covariance=.5)))\nAssertionError: False is not true\n'}, {'test': 'test_eq1d_style_bar_point53m_before_belief_is_preserved (test_boia_p0_geometry.ExclusionCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 57, in test_eq1d_style_bar_point53m_before_belief_is_preserved\n    self.assertTrue(bool(retained((5.47, 0.), covariance=.01)))\nAssertionError: False is not true\n'}, {'test': 'test_false_association_on_bar_has_no_identity_privilege (test_boia_p0_geometry.ExclusionCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 63, in test_false_association_on_bar_has_no_identity_privilege\n    self.assertTrue(bool(retained((3., .2), mean=(3., .2), covariance=.01)))\nAssertionError: False is not true\n'}, {'test': 'test_large_belief_covariance_cannot_erase_far_foreground (test_boia_p0_geometry.ExclusionCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 60, in test_large_belief_covariance_cannot_erase_far_foreground\n    self.assertTrue(bool(retained((3., 0.), covariance=4.)))\nAssertionError: False is not true\n'}, {'test': 'test_near_foreground_bar_is_not_erased_by_zero_covariance_gate (test_boia_p0_geometry.ExclusionCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 53, in test_near_foreground_bar_is_not_erased_by_zero_covariance_gate\n    self.assertTrue(bool(retained((5.8, 0.))))\nAssertionError: False is not true\n'}, {'test': 'test_positive_actual_target_is_retained_with_abstention_tradeoff (test_boia_p0_geometry.ExclusionCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 48, in test_positive_actual_target_is_retained_with_abstention_tradeoff\n    self.assertTrue(bool(retained((5.85, 0.))))\nAssertionError: False is not true\n'}, {'test': 'test_return_order_invalid_mask_and_source_arrays_are_preserved (test_boia_p0_geometry.ExclusionCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 75, in test_return_order_invalid_mask_and_source_arrays_are_preserved\n    self.assertTrue(torch.equal(result, valid))\nAssertionError: False is not true\n'}, {'test': 'test_default_declared_four_rows_use_horizontal_first_hit (test_boia_p0_geometry.HorizontalFreeCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 88, in test_default_declared_four_rows_use_horizontal_first_hit\n    self.assertAlmostEqual(float(radius[0, 36]), expected, places=12)\nAssertionError: 5.0 != 4.698463103929543 within 12 places (0.3015368960704574 difference)\n'}, {'test': 'test_row_minimum_is_taken_after_projection_not_before (test_boia_p0_geometry.HorizontalFreeCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 98, in test_row_minimum_is_taken_after_projection_not_before\n    self.assertAlmostEqual(float(radius[0, 36]), 5 * math.cos(math.radians(20)) - .1, places=12)\nAssertionError: 4.800000000000001 != 4.598463103929543 within 12 places (0.20153689607045777 difference)\n'}, {'test': 'test_undeclared_nonstandard_rows_fail_closed (test_boia_p0_geometry.HorizontalFreeCounterexamples)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 111, in test_undeclared_nonstandard_rows_fail_closed\n    geom.known_free(torch.ones(1, 72, dtype=DT), angles(), torch.zeros(1, 3, dtype=DT),\nAssertionError: ValueError not raised\n'}, {'test': "test_camera_offset_los_counterexample_forecast_and_bpod (test_boia_p0_geometry.VisibilityOriginsAndModelLimits) (method='forecast')", 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 144, in test_camera_offset_los_counterexample_forecast_and_bpod\n    self.assertEqual(float(value), 0.)\nAssertionError: 1.0 != 0.0\n'}, {'test': "test_camera_offset_los_counterexample_forecast_and_bpod (test_boia_p0_geometry.VisibilityOriginsAndModelLimits) (method='bpod')", 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 144, in test_camera_offset_los_counterexample_forecast_and_bpod\n    self.assertEqual(float(value), 0.)\nAssertionError: 1.0 != 0.0\n'}, {'test': 'test_yaw_rotated_camera_origin_and_no_double_offset (test_boia_p0_geometry.VisibilityOriginsAndModelLimits)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 155, in test_yaw_rotated_camera_origin_and_no_double_offset\n    self.assertEqual(float(vis.forecast(pos, yaw, sample, points, valid, None, camera, params)), 0.)\nAssertionError: 1.0 != 0.0\n'}] assertion-failure entries; [{'test': 'test_explicit_zero_and_positive_negative_elevations (test_boia_p0_geometry.HorizontalFreeCounterexamples) (alpha=0.0)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 104, in test_explicit_zero_and_positive_negative_elevations\n    radius = geom.known_free(rho, angles(), torch.zeros(1, 3, dtype=DT),\nTypeError: known_free() got an unexpected keyword argument \'elevations\'\n'}, {'test': 'test_explicit_zero_and_positive_negative_elevations (test_boia_p0_geometry.HorizontalFreeCounterexamples) (alpha=10.0)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 104, in test_explicit_zero_and_positive_negative_elevations\n    radius = geom.known_free(rho, angles(), torch.zeros(1, 3, dtype=DT),\nTypeError: known_free() got an unexpected keyword argument \'elevations\'\n'}, {'test': 'test_explicit_zero_and_positive_negative_elevations (test_boia_p0_geometry.HorizontalFreeCounterexamples) (alpha=20.0)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 104, in test_explicit_zero_and_positive_negative_elevations\n    radius = geom.known_free(rho, angles(), torch.zeros(1, 3, dtype=DT),\nTypeError: known_free() got an unexpected keyword argument \'elevations\'\n'}, {'test': 'test_explicit_zero_and_positive_negative_elevations (test_boia_p0_geometry.HorizontalFreeCounterexamples) (alpha=-10.0)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 104, in test_explicit_zero_and_positive_negative_elevations\n    radius = geom.known_free(rho, angles(), torch.zeros(1, 3, dtype=DT),\nTypeError: known_free() got an unexpected keyword argument \'elevations\'\n'}, {'test': 'test_explicit_zero_and_positive_negative_elevations (test_boia_p0_geometry.HorizontalFreeCounterexamples) (alpha=-20.0)', 'traceback': 'Traceback (most recent call last):\n  File "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/tests/test_boia_p0_geometry.py", line 104, in test_explicit_zero_and_positive_negative_elevations\n    radius = geom.known_free(rho, angles(), torch.zeros(1, 3, dtype=DT),\nTypeError: known_free() got an unexpected keyword argument \'elevations\'\n'}] errors from missing new explicit-elevation API. These are subtest entries, not independent methods. The baseline source hashes were checked before/after. See [base receipt](geometry_base_reproduction.json) and [base log](geometry_base_tests.txt).

B05 isolated original-source run: 2 methods; 22 failing subtest entries; 0 errors. It separately checks point and segment queries, including an unchanged valid environment. See [B05 receipt](geometry_B05_base_reproduction.json) and [B05 log](geometry_B05_base_tests.txt).

Final focused suite: 23 methods, 0 failures, 0 errors in 0.016960s. See [final log](geometry_final_tests.txt), [numerical comparison](geometry_comparison.json) and [machine-readable contracts/source hashes](geometry_audit.json).

Independent arithmetic uses scalar segment projection, rho*cos(alpha), hand-computed joint-path unions and scalar Gaussian erf, rather than calling the source function as its own oracle. Test-only object roles are annotations; online inputs contain no identities. NaN/near-zero range and source-array preservation checks are included.

An initial helper probe on the exact FOV boundary (1,1) at hfov=pi/2 returned false because tan(pi/4) rounds below1. This is retained as a numerical boundary limitation; the smoke fixture now probes .01m inside/outside. No source boundary threshold was changed.

No simulator, GPU, raw evidence or checkpoint was loaded. Source hashes in this receipt bind only this owned unit. Parent integration owns full suite/import firewall checks, protected-path integrity, WORKLOG and local commit. No commit or push was performed by this sub-agent.
