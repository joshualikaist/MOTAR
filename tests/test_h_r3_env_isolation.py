"""Independent float64 geometry oracle and actual Warp CPU BVH regression.

The NumPy oracle never calls production/Warp/Torch transform or intersection helpers.
"""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest

import numpy as np
import torch
import warp as wp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from h_r3_env_isolation import WarpResetIsolation, transform_xyzw

wp.init()


@wp.kernel
def ray_query(mesh: wp.uint64, origins: wp.array(dtype=wp.vec3),
              directions: wp.array(dtype=wp.vec3), out: wp.array(dtype=float)):
    i = wp.tid()
    hit = wp.mesh_query_ray(mesh, origins[i], directions[i], 100.0)
    out[i] = -1.0
    if hit.result:
        out[i] = hit.t


def oracle_transform(vertices, quaternion, translation):
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    # Matrix form of quaternion action; unit q is an SE(3) rotation. Non-unit inputs
    # are tested separately as the legacy representation contract, without normalization.
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=np.float64)
    return np.asarray(vertices, dtype=np.float64) @ r.T + np.asarray(translation, dtype=np.float64)


def oracle_ray(vertices, faces, origin, direction):
    """Independent Moller-Trumbore all triangles, double sided, first positive hit."""
    hits = []
    origin, direction = np.asarray(origin, dtype=np.float64), np.asarray(direction, dtype=np.float64)
    for face in faces:
        a, b, c = np.asarray(vertices, dtype=np.float64)[face]
        e1, e2 = b-a, c-a
        h = np.cross(direction, e2)
        determinant = np.dot(e1, h)
        if abs(determinant) < 1e-12:
            continue
        s = origin-a
        u = np.dot(s, h) / determinant
        q = np.cross(s, e1)
        v = np.dot(direction, q) / determinant
        t = np.dot(e2, q) / determinant
        if -1e-12 <= u <= 1+1e-12 and v >= -1e-12 and u+v <= 1+1e-12 and 0 <= t <= 100:
            hits.append(t)
    return min(hits) if hits else -1.


def box():
    vertices = np.array([[-.5,-.5,-.5],[.5,-.5,-.5],[.5,.5,-.5],[-.5,.5,-.5],
                         [-.5,-.5,.5],[.5,-.5,.5],[.5,.5,.5],[-.5,.5,.5]], dtype=np.float32)
    faces = np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                      [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]], dtype=np.int32)
    return vertices, faces


def fixture():
    """3 envs, 3 PhysX actors/env, unequal mesh vertices and 1/2 mesh assets/env."""
    v, f = box()
    pyramid_v = np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0],[0,0,1]], dtype=np.float32)
    pyramid_f = np.array([[0,2,1],[0,3,2],[0,1,4],[1,2,4],[2,3,4],[3,0,4]], dtype=np.int32)
    source = [types.SimpleNamespace(vertices=np.concatenate([v,v]), faces=np.concatenate([f,f+8])),
              types.SimpleNamespace(vertices=v.copy(), faces=f.copy()),
              types.SimpleNamespace(vertices=pyramid_v, faces=pyramid_f)]
    original = torch.from_numpy(np.concatenate([x.vertices for x in source])).clone()
    updated = torch.zeros_like(original)
    mapping = torch.tensor([1]*8 + [2]*8 + [4]*8 + [7]*5)
    roots = torch.zeros((9,13), dtype=torch.float32)
    roots[:,6] = 1
    roots[1,:3] = torch.tensor([3.,0.,0.]); roots[2,:3] = torch.tensor([8.,0.,0.])
    roots[4,:3] = torch.tensor([4.,0.,0.]); roots[7,:3] = torch.tensor([5.,0.,0.])
    meshes, offset = [], 0
    for s in source:
        faces = torch.tensor(s.faces.flatten(), dtype=torch.int32)
        m = wp.Mesh(points=wp.from_torch(updated[offset:offset+len(s.vertices)], dtype=wp.vec3),
                    indices=wp.from_torch(faces, dtype=wp.int32))
        meshes.append(m)
        offset += len(s.vertices)
    w = types.SimpleNamespace(num_envs=3, vertex_maps_per_env_updated=updated,
           VERTEX_MAPS_PER_ENV_ORIGINAL=original, CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR=mapping,
           unfolded_env_vec_root_tensor=roots, warp_mesh_per_env=meshes,
           global_env_mesh_list=source, global_vertex_counter=len(original),
           CONST_WARP_MESH_ID_LIST=[m.id for m in meshes])
    def legacy(ids):
        w.vertex_maps_per_env_updated[:] = transform_xyzw(roots[mapping,3:7],roots[mapping,:3],original)
        for i in ids:
            meshes[int(i)].refit()
    w.reset_idx = legacy
    return w


def query(mesh, origins):
    o = wp.array(np.asarray(origins, dtype=np.float32), dtype=wp.vec3, device='cpu')
    directions = wp.array(np.tile([1.,0.,0.], (len(origins),1)).astype(np.float32),dtype=wp.vec3,device='cpu')
    out = wp.empty(len(origins), dtype=float, device='cpu')
    wp.launch(ray_query, dim=len(origins), inputs=[mesh.id,o,directions,out], device='cpu')
    return out.numpy().copy()


def independent_world(w, env):
    counts = [len(s.vertices) for s in w.global_env_mesh_list]
    lo, hi = sum(counts[:env]), sum(counts[:env+1])
    mapping = w.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR.numpy()
    original = w.VERTEX_MAPS_PER_ENV_ORIGINAL.numpy()
    roots = w.unfolded_env_vec_root_tensor.numpy()
    return np.asarray([oracle_transform(original[j:j+1],roots[mapping[j],3:7],roots[mapping[j],:3])[0]
                       for j in range(lo,hi)])


class GeometryQualification(unittest.TestCase):
    def assert_world(self, w, env):
        expected = independent_world(w, env)
        begin = sum(len(s.vertices) for s in w.global_env_mesh_list[:env])
        actual = w.vertex_maps_per_env_updated[begin:begin+len(expected)].numpy()
        self.assertLessEqual(float(np.max(np.abs(actual-expected))), 2e-6)
        origins = [[-2,0,.1],[-2,.17,.21],[-2,4,.1],[-2,8,.1]]
        got = query(w.warp_mesh_per_env[env], origins)
        want = [oracle_ray(expected,w.global_env_mesh_list[env].faces,o,[1,0,0]) for o in origins]
        np.testing.assert_allclose(got,want,atol=2e-6,rtol=0)

    def test_all_env_initial_reset_actual_bvh_and_mapping(self):
        w=fixture(); r=WarpResetIsolation(w,3); w.reset_idx(torch.arange(3))
        self.assertEqual(r.ranges,[(0,16),(16,24),(24,29)])
        for env in range(3): self.assert_world(w,env)
        self.assertEqual(r.events[-1]['refit_env_ids'],[0,1,2])

    def test_pose_quaternion_other_reset_preserves_unselected_vertex_and_ray_bytes(self):
        w=fixture(); r=WarpResetIsolation(w,3); w.reset_idx(torch.arange(3))
        before=w.vertex_maps_per_env_updated.clone(); rays=query(w.warp_mesh_per_env[0],[[-2,0,.1],[-2,4,.1]])
        w.unfolded_env_vec_root_tensor[1,1]=4
        w.unfolded_env_vec_root_tensor[1,3:7]=torch.tensor([0.,0.,np.sin(.37),np.cos(.37)])
        w.unfolded_env_vec_root_tensor[4,0]=6
        w.reset_idx(torch.tensor([1]))
        self.assertTrue(torch.equal(before[:16],w.vertex_maps_per_env_updated[:16]))
        self.assertTrue(torch.equal(before[24:],w.vertex_maps_per_env_updated[24:]))
        np.testing.assert_array_equal(rays,query(w.warp_mesh_per_env[0],[[-2,0,.1],[-2,4,.1]]))
        self.assert_world(w,1)
        self.assertEqual(r.events[-1]['refit_env_ids'],[1])
        self.assertEqual(r.events[-1]['vertices'][0]['changed_vertices'],0)

    def test_tiny_quaternion_normalization_changes_actual_legacy_ray_but_fix_isolates(self):
        changes=[]
        for mode in ('legacy_audit','selected'):
            w=fixture(); r=WarpResetIsolation(w,3,mode=mode)
            w.unfolded_env_vec_root_tensor[1,3:7]=torch.tensor([.001,.002,.003,.9999])
            w.reset_idx(torch.arange(3)); before=w.vertex_maps_per_env_updated.clone()
            raybefore=query(w.warp_mesh_per_env[0],[[-2,.11,.13]])
            q=w.unfolded_env_vec_root_tensor[1,3:7]; q/=torch.linalg.norm(q)
            w.reset_idx(torch.tensor([1]))
            rayafter=query(w.warp_mesh_per_env[0],[[-2,.11,.13]])
            changes.append((torch.equal(before[:16],w.vertex_maps_per_env_updated[:16]),np.array_equal(raybefore,rayafter)))
        self.assertEqual(changes,[(False,False),(True,True)])

    def test_selected_pose_quaternion_updates_points_and_actual_bvh(self):
        w=fixture(); r=WarpResetIsolation(w,3); w.reset_idx(torch.arange(3))
        w.unfolded_env_vec_root_tensor[1,1]=4
        w.unfolded_env_vec_root_tensor[1,3:7]=torch.tensor([0.,0.,np.sin(.21),np.cos(.21)])
        w.reset_idx(torch.tensor([0]))
        self.assert_world(w,0)
        got=query(w.warp_mesh_per_env[0],[[-2,4,.1]])[0]
        self.assertGreater(got,0)
        self.assertEqual(r.events[-1]['refit_env_ids'],[0])

    def test_legacy_negative_global_write_stale_bvh_and_ray_miss(self):
        w=fixture(); r=WarpResetIsolation(w,3,mode='legacy_audit'); w.reset_idx(torch.arange(3))
        before=w.vertex_maps_per_env_updated.clone()
        w.unfolded_env_vec_root_tensor[1,1]=4
        w.unfolded_env_vec_root_tensor[1,3:7]=torch.tensor([0.,0.,np.sin(.21),np.cos(.21)])
        w.reset_idx(torch.tensor([1]))
        self.assertFalse(torch.equal(before[:16],w.vertex_maps_per_env_updated[:16]))
        self.assertGreater(r.events[-1]['vertices'][0]['changed_vertices'],0)
        oracle=oracle_ray(independent_world(w,0),w.global_env_mesh_list[0].faces,[-2,4,.1],[1,0,0])
        stale=query(w.warp_mesh_per_env[0],[[-2,4,.1]])[0]
        self.assertGreater(oracle,0); self.assertEqual(stale,-1)
        w.warp_mesh_per_env[0].refit()
        self.assertAlmostEqual(float(query(w.warp_mesh_per_env[0],[[-2,4,.1]])[0]),oracle,places=5)

    def test_order_batch_split_and_empty_reset_invariance(self):
        values=[]
        for batches in ([[2,0,1]],[[1,2,0]],[[0],[2],[1]]):
            w=fixture(); r=WarpResetIsolation(w,3)
            w.unfolded_env_vec_root_tensor[1,1]=4
            w.unfolded_env_vec_root_tensor[7,3:7]=torch.tensor([0.,0.,.2,np.sqrt(.96)])
            for ids in batches: w.reset_idx(torch.tensor(ids))
            before=w.vertex_maps_per_env_updated.clone(); w.reset_idx(torch.tensor([],dtype=torch.int64))
            self.assertTrue(torch.equal(before,w.vertex_maps_per_env_updated))
            values.append(before.numpy())
            for env in range(3): self.assert_world(w,env)
        np.testing.assert_array_equal(values[0],values[1]);np.testing.assert_array_equal(values[0],values[2])

    def test_nonunit_quaternion_legacy_own_reset_contract_preserved(self):
        a,b=fixture(),fixture(); WarpResetIsolation(a,3)
        for w in (a,b): w.unfolded_env_vec_root_tensor[1,3:7]=torch.tensor([.01,.02,.03,.999])
        a.reset_idx(torch.arange(3)); b.reset_idx(torch.arange(3))
        self.assertTrue(torch.equal(a.vertex_maps_per_env_updated,b.vertex_maps_per_env_updated))
        self.assert_world(a,0)

    def test_reject_malformed_mapping_and_mesh_aliases(self):
        for corruption in ('actorenv','actorbounds','count','meshorder','indices','localgeom','publishedids'):
            w=fixture()
            if corruption=='actorenv': w.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR[0]=4
            if corruption=='actorbounds': w.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR[0]=1000
            if corruption=='count': w.global_vertex_counter+=1
            if corruption=='meshorder': w.warp_mesh_per_env.reverse()
            if corruption=='indices': w.global_env_mesh_list[0].faces[0,0]=1
            if corruption=='localgeom': w.VERTEX_MAPS_PER_ENV_ORIGINAL[0,0]+=1
            if corruption=='publishedids': w.CONST_WARP_MESH_ID_LIST.reverse()
            with self.assertRaises(ValueError,msg=corruption): WarpResetIsolation(w,3)

    def test_runtime_mapping_mesh_storage_lifetime_guard(self):
        for corruption in ('mapping','storage','meshes','ids','faces','original','data_storage'):
            w=fixture(); r=WarpResetIsolation(w,3)
            if corruption=='mapping': w.CONST_GLOBAL_VERTEX_TO_ASSET_INDEX_TENSOR[0]=2
            if corruption=='storage': w.vertex_maps_per_env_updated=w.vertex_maps_per_env_updated.clone()
            if corruption=='data_storage': w.vertex_maps_per_env_updated.data=w.vertex_maps_per_env_updated.clone()
            if corruption=='meshes': w.warp_mesh_per_env.reverse()
            if corruption=='ids': w.CONST_WARP_MESH_ID_LIST.reverse()
            if corruption=='faces': w.warp_mesh_per_env[0].indices.assign(np.ones_like(w.warp_mesh_per_env[0].indices.numpy()))
            if corruption=='original': w.VERTEX_MAPS_PER_ENV_ORIGINAL[0,0]+=1
            with self.assertRaises(RuntimeError,msg=corruption): w.reset_idx(torch.tensor([0]))

    def test_bad_reset_ids_fail_before_write_and_close_restores(self):
        w=fixture(); old=w.reset_idx; r=WarpResetIsolation(w,3)
        for ids in ([-1],[3],[0,0],[.5]):
            with self.assertRaises(ValueError): w.reset_idx(ids)
        self.assertEqual(len(r.events),0)
        r.close(); self.assertIs(w.reset_idx,old)

    def test_independent_analytic_box_rotation_first_hit(self):
        v,f=box(); q=[0,0,np.sin(np.pi/8),np.cos(np.pi/8)]
        world=oracle_transform(v,q,[3,0,0])
        self.assertAlmostEqual(oracle_ray(world,f,[0,0,0],[1,0,0]),3-np.sqrt(.5),places=12)
        r=oracle_transform(np.eye(3),q,[0,0,0])
        np.testing.assert_allclose(r@r.T,np.eye(3),atol=1e-15)


if __name__=='__main__': unittest.main()
