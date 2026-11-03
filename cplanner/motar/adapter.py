"""MOTAR 72x4 LiDAR -> P0-R packet arrays (M2; contract in docs/plans/prereg_cplanner_m2_v1.md).

Input is the NORMALISED scan the warp sensor delivers in `obs_dict["depth_range_pixels"]` for one env, shape (4, 72),
row i = elevation 20 - 10 i deg, column j = azimuth 180 - 5 j deg (the `MOTAR-72x4` manifest), BEFORE
`NavRLTask._lidar_distance_m`, which would turn NaN/inf into 12 m and near-clipped into 0 m. The sensor pipeline
(`warp_sensor.py` apply_range_limits / normalize_observation, noise disabled) maps
    no intersection within max_range (kernel miss 1000, or a hit beyond 12 m)  -> 12  -> 1.0
    hit closer than min_range 0.2 m                                           -> -12 -> -1.0
    hit t in [0.2, 12]                                                         -> t / 12
so the status is derived from the value (V1 qualifies this mapping against an independent exact ray caster):
    1.0 -> NO_HIT (range 12.0), -1.0 -> NEAR_CLIPPED, non-finite -> SENSOR_ERROR, otherwise HIT.
HIT ranges are shortened by HIT_MARGIN_M (1 mm) before they are declared: the sensor computes in float32 (V1 dev
sample: range error up to 5.2e-5 m at oblique faces and near-tangent target hits; beam directions differ from the
float64 manifest by up to 1.5e-7 rad), more than the certificate's internal 1e-7 m, and a declared range beyond the
true surface along the manifest beam would let a free ray enter the bar. Shortening only makes free rays shorter
and surfaces nearer. V1 checks on every sampled beam that the declared range does not exceed the exact distance
along the manifest direction.
A HIT whose shortened range falls below min_range is declared NEAR_CLIPPED (the packet is then incomplete).

Header: health RUNTIME (1), noise 0 (the B1-R LiDAR has `enable_sensor_noise = False`), timestamp = step * 0.1 s
(`DECLARED_SIM_SCHEDULE`), sensor position = robot position (nominal_position 0), yaw of the yaw-only attach
(computed by `replay.sensor_yaw` exactly as production does).
"""
import math

import numpy as np

MAX_RANGE = 12.0
MIN_RANGE = 0.2
DT = 0.1
HIT_MARGIN_M = 1.0e-3
SHAPE = (4, 72)
HIT, NO_HIT, SENSOR_ERROR, NEAR_CLIPPED = 1, 2, 3, 5        # core status codes (core.py)
HEALTH_RUNTIME = 1


def derive(norm):
    """(ranges float64 (4, 72), status int8 (4, 72)) from a normalised scan; never raises on values."""
    v = np.array(norm, dtype=np.float64, copy=True).reshape(SHAPE)
    status = np.full(SHAPE, SENSOR_ERROR, np.int8)
    ranges = np.zeros(SHAPE, np.float64)
    finite = np.isfinite(v)
    nohit = finite & (v == 1.0)
    near = finite & (v == -1.0)
    hit = finite & (v > -1.0) & (v < 1.0)
    status[nohit] = NO_HIT
    ranges[nohit] = MAX_RANGE
    r = v * MAX_RANGE - HIT_MARGIN_M
    hit_ok = hit & (r >= MIN_RANGE)
    status[hit_ok] = HIT
    ranges[hit_ok] = r[hit_ok]
    status[(hit & ~hit_ok) | near] = NEAR_CLIPPED
    # values outside [-1, 1] cannot come from the pipeline: SENSOR_ERROR (status already), range 0
    return ranges, status


def packet(norm, step, env, episode, position, yaw):
    """The four P0-R step arrays for one env and one acquisition step."""
    ranges, status = derive(norm)
    header_i = np.array([int(env), int(episode), int(step), HEALTH_RUNTIME, 0], np.int64)
    header_f = np.array([step * DT, float(position[0]), float(position[1]), float(position[2]), float(yaw)],
                        np.float64)
    return ranges, status, header_i, header_f
