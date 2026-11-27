# Environment isolation CPU geometry qualification

Status: PASS, 11 focused tests. No GPU, production edits, original raw changes, or historical causal claim.

Command:
```
PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' /home/fair/miniconda3/envs/aerialgym/bin/python -B -m unittest discover -s tests -p test_h_r3_env_isolation.py -v
```

Independent NumPy float64 quaternion rotation matrices and Moller-Trumbore ray-triangle oracle use manually defined cube/pyramid fixtures with three environments, unequal per-env vertex counts29total (16/8/5), multiple actors, actor index gaps and mesh-local face indices. Oracle analytic rotated cube first hit3−sqrt(.5) is checked to12decimal places. Actual Warp1.0.0 CPU Mesh/refit/mesh_query_ray runs in every geometry/BVH test; no fake counters substitute for ray results.

Negative control: legacy global rewrite updates unselected env0 vertices after its actor pose changes but env1 alone resets/refits. At ray y=4, independent current-root geometry has a valid hit while stale BVH returns−1; explicit env0 refit repairs the hit. Selected-only reset leaves env0 points AND ray values unchanged while env1 selected vertices match float64 geometry and actual BVH rays.

Tiny source-consistent hypothesis: normalizing unselected quaternion[.001,.002,.003,.9999] before another-env reset changes8vertices by up to9.5367431640625e−7m and actual ray depth by4.76837158203125e−7m under legacy. Selected reset preserves both bytes. This demonstrates a sufficient mechanism; historical P02 quaternion/vertex values were not recorded, so historical numerical causation remains unproved until the real N128 diagnostic.

Random1000 unit-quaternion transforms: CPU float32 vs independent float64 max absolute coordinate error6.343943792508355e−7m, mean7.442558035928212e−8m. Scene geometry and Warp ray errors use predeclared2e−6m CPU assertion tolerance; unrelated vertices/rays require exact bytes, without tolerances. Nonunit quaternion own-env legacy arithmetic is preserved without normalization.

Coverage: all-env initialization, selected pose/quaternion updates and refit, unrelated byte preservation, actual stale-BVH negative, tiny normalization negative, batched/split/reordered IDs, empty reset, out-of-range/duplicate/noninteger IDs rejection, malformed actor/env mapping, mesh point alias/topology/published ID mismatch, runtime mapping/storage/mesh IDs/face indices/local vertex replacement/mutation and close/restore behavior.

API: attach_warp_isolation(task, mode='selected'|'legacy_audit') returns recorder; snapshot() returns detached arrays roots/vertices/vertex_actor_index/vertex_env_index/mesh_ids/vertex_ranges/event_count. recorder.events captures exact selected IDs, write ranges, refit mesh IDs and per-env before/after SHA/delta. All this telemetry is evaluation-only and never goes into actor/perception.

The runtime selected implementation takes the original reset method's exact tf_apply callable when available, writes only selected mesh slices in their existing Warp-alias memory and refits those exact meshes; no PhysX reset, RNG, state cache, sensor, checkpoint or tracking semantics are replaced. Global mapping, original vertices, topology, pointers and published sensor mesh IDs are guarded. Shared-state exclusion beyond geometry remains subject to root's N128 evaluator/RNG/kinematic/PhysX diagnostic.
