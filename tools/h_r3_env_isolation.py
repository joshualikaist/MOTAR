"""Instance-only Warp reset isolation for H-R3 mechanics qualification.

This module does not modify production classes, dynamics, RNG, sensor algorithms or GT
inputs. Geometry telemetry is evaluator-only. Install before initial reset. The selected
mode writes only selected meshes' existing point storage and refits those same meshes.
"""
from __future__ import annotations

import hashlib
import numpy as np
import torch

SCHEMA = 'h-r3.environment-isolation.v1'


def _array(t):
    return t.detach().cpu().numpy().copy()


def _sha(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def transform_xyzw(q, t, v):
    """Production-equivalent tf_apply arithmetic, including non-unit representations.

    Deliberately do not normalize: normalization would change own-env reset semantics.
    The independent test oracle constructs a float64 rotation matrix separately.
    """
    cross = 2 * torch.cross(q[:, :3], v, dim=-1)
    return v + q[:, 3:] * cross + torch.cross(q[:, :3], cross, dim=-1) + t


class WarpResetIsolation:
    def __init__(self, warp, actors_per_env, mode='selected'):
        if mode not in ('selected', 'legacy_audit'):
            raise ValueError('mode must be selected or legacy_audit')
        if getattr(warp, '_h_r3_isolation', None) is not None:
            raise RuntimeError('Warp isolation already installed')
        self.warp, self.mode = warp, mode
        self.num_envs, self.actors_per_env = int(warp.num_envs), int(actors_per_env)
        if self.num_envs < 1 or self.actors_per_env < 1:
            raise ValueError('empty actor/environment layout')
        self.original_reset = warp.reset_idx
        # Use the exact production TorchScript callable already bound by WarpEnv. CPU
        # standalone fixtures have no production module/Isaac dependency and use the
        # equivalent arithmetic above, checked independently against float64 geometry.
        reset_function = getattr(self.original_reset, '__func__', self.original_reset)
        self._transform = getattr(reset_function, '__globals__', {}).get('tf_apply', transform_xyzw)
        self.events = []
        self._updated = warp.vertex_maps_per_env_updated
        self._original = warp.VERTEX_MAPS_PER_ENV_ORIGINAL
        self._map = warp.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR
        self._roots = warp.unfolded_env_vec_root_tensor
        self._mesh_objects = tuple(warp.warp_mesh_per_env)
        if self._updated.ndim != 2 or self._updated.shape[1] != 3:
            raise ValueError('expected contiguous vertex triples')
        if not self._updated.is_contiguous() or self._updated.dtype != torch.float32:
            raise ValueError('Warp mesh points require contiguous float32 storage')
        if self._original.shape != self._updated.shape or self._original.dtype != torch.float32:
            raise ValueError('original/updated vertex layouts differ')
        if self._roots.shape != (self.num_envs * self.actors_per_env, 13):
            raise ValueError('root actor layout differs from IGE equal-env contract')
        if self._map.shape != (len(self._updated),) or self._map.dtype not in (torch.int64, torch.int32):
            raise ValueError('invalid vertex-to-actor map')
        if any(t.device != self._updated.device for t in (self._original, self._map, self._roots)):
            raise ValueError('geometry tensor devices differ')
        mapping = _array(self._map).astype(np.int64)
        if np.any(mapping < 0) or np.any(mapping >= len(self._roots)):
            raise ValueError('out-of-range vertex actor')
        if len(warp.global_env_mesh_list) != self.num_envs or len(self._mesh_objects) != self.num_envs:
            raise ValueError('mesh count differs from num_envs')
        self.ranges, offset = [], 0
        for env, (source, mesh) in enumerate(zip(warp.global_env_mesh_list, self._mesh_objects)):
            count = len(source.vertices)
            if count < 1 or offset + count > len(mapping):
                raise ValueError('empty/overrun per-env mesh')
            if np.any(mapping[offset:offset + count] // self.actors_per_env != env):
                raise ValueError('vertex actor belongs to a different environment')
            # Warp points must alias this exact slice: restoring another copy is insufficient.
            expected_ptr = self._updated[offset:offset + count].data_ptr()
            if int(mesh.points.ptr) != expected_ptr or int(mesh.points.shape[0]) != count:
                raise ValueError('Warp mesh point storage does not match per-env vertex slice')
            faces = np.asarray(source.faces, dtype=np.int32).reshape(-1)
            indices = mesh.indices.numpy().reshape(-1)
            if not np.array_equal(indices, faces) or np.any(faces < 0) or np.any(faces >= count):
                raise ValueError('mesh face topology correspondence failure')
            if not np.array_equal(_array(self._original[offset:offset + count]),
                                  np.asarray(source.vertices, dtype=np.float32)):
                raise ValueError('local geometry differs from mesh construction source')
            self.ranges.append((offset, offset + count))
            offset += count
        if offset != len(mapping) or int(warp.global_vertex_counter) != offset:
            raise ValueError('vertex count mismatch')
        if not np.array_equal(np.asarray(warp.CONST_WARP_MESH_ID_LIST, dtype=np.uint64),
                              np.asarray([m.id for m in self._mesh_objects], dtype=np.uint64)):
            raise ValueError('published sensor mesh IDs differ from owned meshes')
        self._tensor_ptrs = tuple(t.data_ptr() for t in (self._updated, self._original, self._map, self._roots))
        self._mapping_sha = _sha(mapping)
        self._indices_sha = [_sha(m.indices.numpy()) for m in self._mesh_objects]
        self._original_sha = _sha(_array(self._original))
        self._topology = [(int(m.id), int(m.points.ptr), int(m.indices.ptr)) for m in self._mesh_objects]
        warp.reset_idx = self.reset_idx
        warp._h_r3_isolation = self

    def _validate_lifetime(self):
        w = self.warp
        if any(a is not b for a, b in ((w.vertex_maps_per_env_updated, self._updated),
                                      (w.VERTEX_MAPS_PER_ENV_ORIGINAL, self._original),
                                      (w.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR, self._map),
                                      (w.unfolded_env_vec_root_tensor, self._roots))):
            raise RuntimeError('geometry storage replaced after isolation installation')
        if tuple(t.data_ptr() for t in (self._updated, self._original, self._map, self._roots)) != self._tensor_ptrs:
            raise RuntimeError('geometry tensor storage pointer changed')
        if len(w.warp_mesh_per_env) != self.num_envs or any(a is not b for a, b in zip(w.warp_mesh_per_env, self._mesh_objects)):
            raise RuntimeError('Warp mesh object replaced')
        current = [(int(m.id), int(m.points.ptr), int(m.indices.ptr)) for m in self._mesh_objects]
        if current != self._topology or _sha(_array(self._map).astype(np.int64)) != self._mapping_sha:
            raise RuntimeError('mesh pointers or actor mapping changed')
        if ([_sha(m.indices.numpy()) for m in self._mesh_objects] != self._indices_sha
                or _sha(_array(self._original)) != self._original_sha):
            raise RuntimeError('mesh face indices or local vertex geometry changed')
        if list(w.CONST_WARP_MESH_ID_LIST) != [m.id for m in self._mesh_objects]:
            raise RuntimeError('sensor mesh IDs changed')

    def _ids(self, ids):
        values = _array(ids).reshape(-1) if isinstance(ids, torch.Tensor) else np.asarray(ids).reshape(-1)
        if values.dtype.kind not in 'iu' or np.any(values < 0) or np.any(values >= self.num_envs):
            raise ValueError('reset IDs must be integer environment indices')
        if len(set(values.tolist())) != len(values):
            raise ValueError('duplicate reset IDs')
        return [int(i) for i in values]

    def reset_idx(self, env_ids):
        self._validate_lifetime()
        ids = self._ids(env_ids)
        before = _array(self._updated)
        if self.mode == 'legacy_audit':
            result = self.original_reset(env_ids)
            write_ranges = self.ranges
        else:
            for env in ids:
                lo, hi = self.ranges[env]
                actor_ids = self._map[lo:hi].long()
                self._updated[lo:hi] = self._transform(self._roots[actor_ids, 3:7],
                                                     self._roots[actor_ids, :3],
                                                     self._original[lo:hi])
            for env in ids:
                self._mesh_objects[env].refit()
            result = None
            write_ranges = [self.ranges[i] for i in ids]
        after = _array(self._updated)
        rows = []
        for env, (lo, hi) in enumerate(self.ranges):
            rows.append({'env': env, 'before_sha256': _sha(before[lo:hi]),
                         'after_sha256': _sha(after[lo:hi]),
                         'changed_vertices': int(np.any(before[lo:hi] != after[lo:hi], axis=1).sum()),
                         'max_abs_delta': float(np.max(np.abs(after[lo:hi] - before[lo:hi])))})
        event = {'mode': self.mode, 'env_ids': ids, 'write_vertex_ranges': [list(x) for x in write_ranges],
                 'refit_env_ids': ids, 'refit_mesh_ids': [int(self._mesh_objects[i].id) for i in ids],
                 'vertices': rows, 'actor_root_sha256': _sha(_array(self._roots))}
        self.events.append(event)
        if self.mode == 'selected' and any(row['changed_vertices'] for row in rows if row['env'] not in ids):
            raise RuntimeError('selected reset mutated unrelated vertex storage')
        return result

    def snapshot(self):
        self._validate_lifetime()
        return {'roots': _array(self._roots), 'vertices': _array(self._updated),
                'vertex_actor_index': _array(self._map),
                'vertex_env_index': _array(self._map) // self.actors_per_env,
                'mesh_ids': np.asarray([m.id for m in self._mesh_objects], dtype=np.uint64),
                'vertex_ranges': np.asarray(self.ranges, dtype=np.int64), 'event_count': len(self.events)}

    def close(self):
        if getattr(self.warp, '_h_r3_isolation', None) is self:
            self.warp.reset_idx = self.original_reset
            del self.warp._h_r3_isolation


def attach_warp_isolation(task, mode='selected'):
    """Attach only to this evaluation task; no evaluator GT feeds online control."""
    manager = task.sim_env
    ige = manager.IGE_env
    return WarpResetIsolation(manager.warp_env, int(ige.num_assets_per_env), mode=mode)
