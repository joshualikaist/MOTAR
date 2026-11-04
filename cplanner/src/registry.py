"""P0-R internal registry: beam manifests, sensor configurations and the static-bar catalog.

Nothing here is supplied by a caller per step. A core is built from a registered configuration NAME; the caller never
passes a direction, a spec object or a shape list (the v3-v7 boundary accepted all three as Python objects).
"""
import math
import types
from collections import namedtuple

import numpy as np

# ---------------------------------------------------------------- static-bar catalog (MOTAR bars_h3)
# Half-sizes (hx, hy) in metres of the 40 upright, full-height (0-3 m), axis-aligned box bars of
# W/resources/models/environment_assets/bars_h3/bar_000..039.urdf (placement roll/pitch/yaw ratios all 0,
# z ratio 0.5: env_object_config.py:812-857). Generated from snapshot/geometry_results.json
# (sha256 e10991b2606a2a4b426a76561c898b0fdd8ab40bc2e2262999c21147161c1112), the same list v3-v7 used
# (`indep.catalog()`); tests/check_catalog.py compares it with the URDF files read-only.
BAR_HALF_SIZES = (
    (0.3548, 0.2878),
    (0.3717, 0.33945),
    (0.21885, 0.3951),
    (0.35225, 0.3572),
    (0.2256, 0.2901),
    (0.27415, 0.38535),
    (0.32875, 0.36455),
    (0.2887, 0.24545),
    (0.3109, 0.21275),
    (0.36555, 0.32635),
    (0.3516, 0.2709),
    (0.39415, 0.3786),
    (0.3557, 0.23895),
    (0.29335, 0.20875),
    (0.23085, 0.3366),
    (0.34895, 0.3935),
    (0.26515, 0.2741),
    (0.2939, 0.2379),
    (0.226, 0.29515),
    (0.2454, 0.33395),
    (0.28745, 0.36655),
    (0.34005, 0.26245),
    (0.36645, 0.36095),
    (0.2775, 0.25765),
    (0.3365, 0.22795),
    (0.24, 0.20145),
    (0.3574, 0.33295),
    (0.34105, 0.35615),
    (0.2918, 0.31375),
    (0.22795, 0.2229),
    (0.3337, 0.2942),
    (0.31305, 0.353),
    (0.32695, 0.3107),
    (0.31185, 0.2608),
    (0.20615, 0.28735),
    (0.2429, 0.2817),
    (0.3707, 0.2468),
    (0.21165, 0.2563),
    (0.2587, 0.3324),
    (0.3114, 0.3568),
)
BAR_Z_EXTENT = (0.0, 3.0)
TARGET_RADIUS = 0.20            # MOTAR analytic target sphere (navrl_lidar_config.target_radius)

# ---------------------------------------------------------------- beam manifests
Layout = namedtuple("Layout", "name rows_deg az0_deg daz_deg n_az planar")

LAYOUTS = types.MappingProxyType({
    # v3-v7 synthetic planar ring: beam j at 5 deg x j (identical directions to P0 v3-v7).
    "P0R-72x1": Layout("P0R-72x1", (0.0,), 0.0, 5.0, 72, True),
    # MOTAR: row i at 20 - 10 i deg (+20, +10, 0, -10), column j at 180 - 5 j deg (+180 ... -175, decreasing),
    # flattened id = 72 i + j (warp_lidar.py:76-84, navrl_lidar_config.py:60-61, navrl_task.py:6017).
    "MOTAR-72x4": Layout("MOTAR-72x4", (20.0, 10.0, 0.0, -10.0), 180.0, -5.0, 72, False),
})


def directions(layout):
    """Unit beam directions (rows, cols, 3) in the yaw-level sensor frame, as a read-only float64 array."""
    lay = LAYOUTS[layout]
    out = np.empty((len(lay.rows_deg), lay.n_az, 3), dtype=np.float64)
    for i, e in enumerate(lay.rows_deg):
        for j in range(lay.n_az):
            a = math.radians(lay.az0_deg + lay.daz_deg * j)
            er = math.radians(e)
            out[i, j] = (math.cos(a) * math.cos(er), math.sin(a) * math.cos(er), math.sin(er))
    out.flags.writeable = False
    return out


# ---------------------------------------------------------------- sensor/certificate configurations
Config = namedtuple("Config", "name layout min_range max_range no_hit_fill range_error_bound dt "
                              "allow_synthetic_health nearest_hit_assured plan_radius resolution max_nodes "
                              "orientation_prior arena map_bounds coverage direction_error_m")

_BASE = dict(min_range=0.2, max_range=12.0, no_hit_fill=12.0, range_error_bound=0.0, dt=0.1,
             allow_synthetic_health=True, nearest_hit_assured=True, plan_radius=0.45, resolution=0.002,
             max_nodes=100000, orientation_prior="AXIS_ALIGNED_CATALOG", arena=None, map_bounds=(-16.0, 16.0),
             coverage="VERBATIM_PY", direction_error_m=0.0)
# coverage: VERBATIM_PY = coverage_b._coverage_B (qualified, R2'); NATIVE_EXACT = coverage_native.coverage_B_native,
# the exact C port (M3 candidate; qualified only by the M3 equivalence gate, docs/plans/prereg_cplanner_m3_v1.md).
# direction_error_m: declared bound eps on the lateral offset, at any range up to max_range, between a beam's manifest
# direction and the ray the sensor actually cast (0.0: directions exact, the R2' behaviour). With eps > 0 the core
# certifies against bar footprints shrunk by eps with the radius grown by 2 eps, and shrinks the bar-height interval
# by eps (docs/plans/prereg_cplanner_dirbound_v1.md).


def _cfg(name, **kw):
    v = dict(_BASE, **kw)
    return Config(name=name, **v)


CONFIGS = types.MappingProxyType({c.name: c for c in (
    _cfg("SYNTH-72x1", layout="P0R-72x1"),
    _cfg("SYNTH-72x1-NOASSURE", layout="P0R-72x1", nearest_hit_assured=False),
    _cfg("SYNTH-72x1-ERRBOUND", layout="P0R-72x1", range_error_bound=0.02),      # R-6: never qualified
    _cfg("SYNTH-72x4", layout="MOTAR-72x4"),
    _cfg("SYNTH-72x4-RUNTIME", layout="MOTAR-72x4", allow_synthetic_health=False),
    # MOTAR runtime: runtime-declared health only; nearest-hit assurance NOT declared until V1 qualifies the warp
    # near-range semantics; arena 0..40 m in x and y (navrl_bars_env.py:104-107).
    _cfg("MOTAR-72x4-RUNTIME", layout="MOTAR-72x4", allow_synthetic_health=False, nearest_hit_assured=False,
         arena=(0.0, 40.0, 0.0, 40.0), map_bounds=(-2.0, 42.0)),
    # M3: the same configuration with the native coverage port (identical results required by the M3 gate).
    _cfg("MOTAR-72x4-RUNTIME-NATIVE", layout="MOTAR-72x4", allow_synthetic_health=False, nearest_hit_assured=False,
         arena=(0.0, 40.0, 0.0, 40.0), map_bounds=(-2.0, 42.0), coverage="NATIVE_EXACT"),
    # Direction-bound repair (M2 V1 FAIL, D-2026-11-04-2): the MOTAR kernel casts float32 rays that differ from the
    # float64 manifest by ~1e-7 rad; eps = 1 mm bounds the lateral offset with a wide margin.
    _cfg("MOTAR-72x4-RUNTIME-DIR", layout="MOTAR-72x4", allow_synthetic_health=False, nearest_hit_assured=False,
         arena=(0.0, 40.0, 0.0, 40.0), map_bounds=(-2.0, 42.0), direction_error_m=1e-3),
    _cfg("MOTAR-72x4-RUNTIME-DIR-NATIVE", layout="MOTAR-72x4", allow_synthetic_health=False, nearest_hit_assured=False,
         arena=(0.0, 40.0, 0.0, 40.0), map_bounds=(-2.0, 42.0), coverage="NATIVE_EXACT", direction_error_m=1e-3),
)})
assert all(0.0 <= c.direction_error_m and all(h > 10 * c.direction_error_m for hs in BAR_HALF_SIZES for h in hs)
           for c in CONFIGS.values())
