"""Warp kernel for E-L0's centre-ray query (module scope, as Warp code generation requires).

Imported lazily by ``motar_eval.e_l0.centre_ray_blocked`` inside the simulator process only.
"""

import warp as wp


@wp.kernel
def centre_ray(mesh_ids: wp.array(dtype=wp.uint64), origins: wp.array(dtype=wp.vec3),
               targets: wp.array(dtype=wp.vec3), blocked: wp.array(dtype=wp.int32)):
    i = wp.tid()
    o = origins[i]
    d = targets[i] - o
    length = wp.length(d)
    blocked[i] = wp.int32(0)
    if length > 1.0e-6:
        rd = d / length
        t = float(0.0)
        u = float(0.0)
        v = float(0.0)
        sign = float(0.0)
        normal = wp.vec3()
        face = int(0)
        if wp.mesh_query_ray(mesh_ids[i], o, rd, length, t, u, v, sign, normal, face):
            blocked[i] = wp.int32(1)
