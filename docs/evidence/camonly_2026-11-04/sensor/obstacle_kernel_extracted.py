import warp as wp


@wp.kernel
def _render_obstacle_depth_kernel(
    mesh_ids: wp.array(dtype=wp.uint64),
    origins: wp.array(dtype=wp.vec3),
    orientations: wp.array(dtype=wp.quat),
    ray_vectors: wp.array2d(dtype=wp.vec3),
    far_plane: float,
    obstacle_depth: wp.array(dtype=float, ndim=3),
):
    """Render full-scene metric depth for every low-resolution camera ray."""
    env_id, row, col = wp.tid()
    ro = origins[env_id]
    rd = wp.normalize(wp.quat_rotate(orientations[env_id], ray_vectors[row, col]))
    t = float(0.0)
    u = float(0.0)
    v = float(0.0)
    sign = float(0.0)
    normal = wp.vec3()
    face = int(0)
    dist = far_plane
    if wp.mesh_query_ray(mesh_ids[env_id], ro, rd, far_plane, t, u, v, sign, normal, face):
        dist = t
    obstacle_depth[env_id, row, col] = dist
