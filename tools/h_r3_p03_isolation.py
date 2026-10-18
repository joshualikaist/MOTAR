"""Opt-in P03 evaluation geometry isolation with audit/runtime instrumentation.

Both modes retain ISO-Q02's exact selected-only transform/refit semantics. Runtime
keeps immutable references and the expected vertex buffer on the existing device;
every integrity check compares their bits, without vertex downloads or hashes.
Explicit ``snapshot()`` remains available for registered diagnostic windows only.
Nothing here changes a production class or supplies geometry to online control.
"""
from __future__ import annotations

import torch

try:
    from tools.h_r3_env_isolation import WarpResetIsolation
except ModuleNotFoundError:
    from h_r3_env_isolation import WarpResetIsolation

SCHEMA = 'h-r3.p03-isolation.v1'


def _signature(tensor):
    return (tensor.data_ptr(), tuple(tensor.shape), tuple(tensor.stride()),
            tensor.dtype, tensor.device)


def _bits(tensor):
    """Expose exact float32 bits, including signed zero and NaN payloads."""
    return tensor.view(torch.int32) if tensor.dtype == torch.float32 else tensor


class _DeviceGuards:
    def _initialize_device_guards(self):
        import warp as wp

        tensors = (self._updated, self._original, self._map, self._roots)
        if any(not value.is_contiguous() for value in tensors):
            raise ValueError('runtime isolation requires contiguous geometry storage')
        self._storage_signatures = tuple(_signature(value) for value in tensors)
        self._expected_vertices = self._updated.clone()
        self._original_reference = self._original.clone()
        self._mapping_reference = self._map.clone()
        self._index_views = tuple(wp.to_torch(mesh.indices) for mesh in self._mesh_objects)
        if any(value.device != self._updated.device or not value.is_contiguous()
               for value in self._index_views):
            raise ValueError('mesh indices must share the contiguous geometry device')
        self._index_references = tuple(value.clone() for value in self._index_views)
        self._index_signatures = tuple(_signature(value) for value in self._index_views)
        self._mesh_shapes = tuple((tuple(mesh.points.shape), tuple(mesh.indices.shape))
                                  for mesh in self._mesh_objects)
        self.integrity_checks = 0
        self._validate_device_lifetime()

    def _validate_device_lifetime(self):
        w = self.warp
        actual = (w.vertex_maps_per_env_updated, w.VERTEX_MAPS_PER_ENV_ORIGINAL,
                  w.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR, w.unfolded_env_vec_root_tensor)
        owned = (self._updated, self._original, self._map, self._roots)
        if any(a is not b for a, b in zip(actual, owned)):
            raise RuntimeError('geometry storage replaced after isolation installation')
        if tuple(_signature(value) for value in owned) != self._storage_signatures:
            raise RuntimeError('geometry tensor storage/layout changed')
        if int(w.num_envs) != self.num_envs or int(w.global_vertex_counter) != len(self._updated):
            raise RuntimeError('environment or vertex count changed')
        if len(w.warp_mesh_per_env) != self.num_envs or any(
                a is not b for a, b in zip(w.warp_mesh_per_env, self._mesh_objects)):
            raise RuntimeError('Warp mesh object replaced')
        topology = [(int(mesh.id), int(mesh.points.ptr), int(mesh.indices.ptr))
                    for mesh in self._mesh_objects]
        shapes = tuple((tuple(mesh.points.shape), tuple(mesh.indices.shape))
                       for mesh in self._mesh_objects)
        if topology != self._topology or shapes != self._mesh_shapes:
            raise RuntimeError('mesh pointers, refit mapping or topology layout changed')
        if list(w.CONST_WARP_MESH_ID_LIST) != [mesh.id for mesh in self._mesh_objects]:
            raise RuntimeError('sensor mesh IDs changed')
        if tuple(_signature(value) for value in self._index_views) != self._index_signatures:
            raise RuntimeError('mesh index storage changed')
        # All reductions remain on the existing device; one scalar transfer gates
        # the operation. Unlike float equality, the int32 view rejects -0/+0 changes.
        checks = [torch.eq(_bits(self._updated), _bits(self._expected_vertices)).all(),
                  torch.eq(_bits(self._original), _bits(self._original_reference)).all(),
                  torch.eq(self._map, self._mapping_reference).all(),
                  torch.isfinite(self._updated).all()]
        checks.extend(torch.eq(a, b).all()
                      for a, b in zip(self._index_views, self._index_references))
        if not bool(torch.stack(checks).all().item()):
            raise RuntimeError('geometry bits, immutable local vertices, mapping or topology mutated')
        self.integrity_checks += 1

    def check_integrity(self):
        """Exact whole-device guard, not a lossy hash or sampled vertex check."""
        self._validate_lifetime()
        return {'schema': SCHEMA, 'mode': self.instrumentation_mode, 'status': 'PASS',
                'bitwise_geometry': True, 'immutable_topology': True,
                'vertex_count': len(self._updated), 'mesh_count': self.num_envs,
                'reset_event_count': len(self.events), 'checks': self.integrity_checks}


class RuntimeIsolation(_DeviceGuards, WarpResetIsolation):
    """Same selected-only reset, compact chronology and exact device guards."""

    def __init__(self, warp, actors_per_env):
        super().__init__(warp, actors_per_env, mode='selected')
        self.instrumentation_mode = 'runtime'
        self._initialize_device_guards()

    def _validate_lifetime(self):
        self._validate_device_lifetime()

    def reset_idx(self, env_ids):
        self._validate_lifetime()
        ids = self._ids(env_ids)
        for env in ids:
            lo, hi = self.ranges[env]
            actor_ids = self._map[lo:hi].long()
            world = self._transform(self._roots[actor_ids, 3:7],
                                    self._roots[actor_ids, :3], self._original[lo:hi])
            self._updated[lo:hi] = world
            self._expected_vertices[lo:hi] = world
        for env in ids:
            self._mesh_objects[env].refit()
        self._validate_lifetime()
        self.events.append({'mode': 'selected', 'instrumentation_mode': 'runtime',
                            'env_ids': ids, 'write_vertex_ranges': [list(self.ranges[i]) for i in ids],
                            'refit_env_ids': ids,
                            'refit_mesh_ids': [int(self._mesh_objects[i].id) for i in ids],
                            'unselected_vertices_bitwise_preserved': True,
                            'integrity_checks': self.integrity_checks})


class AuditIsolation(_DeviceGuards, WarpResetIsolation):
    """ISO-Q02 detailed reset snapshots, plus runtime-equivalent frame guards."""

    def __init__(self, warp, actors_per_env):
        super().__init__(warp, actors_per_env, mode='selected')
        self.instrumentation_mode = 'audit'
        self._initialize_device_guards()

    def _validate_lifetime(self):
        super()._validate_lifetime()
        self._validate_device_lifetime()

    def reset_idx(self, env_ids):
        result = super().reset_idx(env_ids)
        # The original audited implementation independently checks every untouched
        # vertex and records actual before/after data. Accept only selected writes.
        for env in self.events[-1]['env_ids']:
            lo, hi = self.ranges[env]
            self._expected_vertices[lo:hi] = self._updated[lo:hi]
        self._validate_lifetime()
        self.events[-1]['instrumentation_mode'] = 'audit'
        self.events[-1]['unselected_vertices_bitwise_preserved'] = True
        return result


def attach(task, mode='runtime'):
    """Install on one evaluation task; production WarpEnv source is untouched."""
    modes = {'audit': AuditIsolation, 'runtime': RuntimeIsolation}
    if mode not in modes:
        raise ValueError('mode must be audit or runtime')
    manager = task.sim_env
    return modes[mode](manager.warp_env, int(manager.IGE_env.num_assets_per_env))
