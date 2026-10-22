"""Q03 local obstacle map v0 CPU tests: synthetic ray-cast scenes from the Q02-A production-faithful helper.

They check the map's contract against known geometry (free / occupied / unknown / conflict, staleness, ego-motion,
near-clip warnings); they do not measure mapping accuracy in the simulator.
"""
import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q02_evidence as q2a
from motar_eval.q02 import contracts as c, obstacle_map as om, validation as va


def frame_at(world_bars, *, pos_xy=(0.0, 0.0), yaw=0.0, stamp=0.0, **kw):
    """A frame seen from vehicle pose (pos_xy, yaw) of bars given in the world frame (cx, cy, r)."""
    pos = np.array([pos_xy[0], pos_xy[1], q2a.POS[2]])
    cs, sn = math.cos(yaw), math.sin(yaw)
    bars = tuple((cs * (x - pos[0]) + sn * (y - pos[1]), -sn * (x - pos[0]) + cs * (y - pos[1]), r) for x, y, r in world_bars)
    clock = c.Clock(stamp, stamp, "DECLARED_SIM_SCHEDULE")
    inp = q2a.make(render_target=False, bars=bars, yaw=yaw, position=pos, decision=stamp, cam_clock=clock, lidar_clock=clock, **kw)
    return inp, va.validate(inp, q2a.CAM, q2a.LID)


def built(world_bars=(), **kw):
    m = om.LocalObstacleMap()
    inp, rep = frame_at(world_bars, **kw)
    receipt = m.integrate(inp, rep)
    return m, receipt


class LocalMapTests(unittest.TestCase):
    def test_empty_space_is_free_only_where_observed(self):
        m, r = built()
        self.assertTrue(r.accepted); self.assertEqual(r.occupied_cells, 0)
        self.assertEqual(m.state_at((3.0, 0.0), 0.0), om.FREE)
        self.assertEqual(m.segment_status((0, 0), (3, 0), 0.3, 0.0)[0], om.SAFE)
        self.assertEqual(m.state_at((11.5, 0.0), 0.0), om.FREE)         # LiDAR no-return = no hit within 12 m
        self.assertEqual(m.state_at((0.0, 0.0), 0.0), om.FREE)

    def test_single_bar_occupied_and_space_behind_unknown(self):
        m, r = built(((4.0, 0.0, 0.3),))
        self.assertGreater(r.occupied_cells, 0)
        self.assertEqual(m.state_at((3.75, 0.0), 0.0), om.OCCUPIED)
        self.assertEqual(m.state_at((5.5, 0.0), 0.0), om.UNKNOWN)       # not observed: never free
        self.assertEqual(m.segment_status((0, 0), (6, 0), 0.3, 0.0)[0], om.BLOCKED)
        self.assertEqual(m.line_of_sight((0, 0), (8, 0), 0.0), "BLOCKED")
        self.assertLess(m.clearance((3.0, 0.0), 0.0), 1.0)

    def test_bar_edge_side_path(self):
        m, _ = built(((4.0, 0.6, 0.3),))                                  # bar edge 0.3 m beside the axis
        self.assertEqual(m.segment_status((0, 0), (3.5, -0.3), 0.15, 0.0)[0], om.SAFE)
        self.assertEqual(m.segment_status((0, 0), (5.0, 0.6), 0.15, 0.0)[0], om.BLOCKED)

    def test_narrow_passage_depends_on_body_radius(self):
        m, _ = built(((3.0, 0.8, 0.3), (3.0, -0.8, 0.3)))                 # 1.0 m gap between bar surfaces
        narrow = m.segment_status((0, 0), (2.6, 0), 0.2, 0.0)[0]
        wide = m.segment_status((0, 0), (3.0, 0), 0.6, 0.0)[0]
        self.assertEqual(narrow, om.SAFE); self.assertEqual(wide, om.BLOCKED)

    def test_near_clipped_raises_a_warning_and_carves_nothing(self):
        inp, rep = frame_at(())
        lidar = np.array(inp.lidar).reshape(4, 72); lidar[:, 30:42] = 0.0     # near-clipped beams ahead
        inp2 = c.EvidenceInput.build(**{**{f: getattr(inp, f) for f in inp.__dataclass_fields__}, "lidar": lidar.reshape(-1)})
        m = om.LocalObstacleMap(); r = m.integrate(inp2, va.validate(inp2, q2a.CAM, q2a.LID))
        self.assertEqual(r.near_clipped_beams, 48); self.assertIn("NEAR_CLIPPED_RETURNS", r.reasons)
        self.assertTrue(m.validity(0.0)["near_obstacle_warning"])
        self.assertEqual(m.state_at((-3.0, 0.0), 0.0), om.FREE)           # other beams still carve

    def test_stale_evidence_returns_to_unknown(self):
        m, _ = built(((4.0, 0.0, 0.3),))
        self.assertEqual(m.state_at((2.0, 0.0), 1.5), om.UNKNOWN)         # free ages out (1 s)
        self.assertEqual(m.state_at((3.75, 0.0), 1.5), om.OCCUPIED)       # occupancy kept longer (5 s)
        self.assertEqual(m.state_at((3.75, 0.0), 6.0), om.UNKNOWN)
        self.assertEqual(m.segment_status((0, 0), (2, 0), 0.3, 1.5)[2], ("MAP_STALE",))
        self.assertEqual(m.state_at((2.0, 0.0), -0.5), om.UNKNOWN)        # no evidence from the future

    def test_camera_lidar_disagreement_is_conflict_not_free(self):
        inp, rep = frame_at(())
        lidar = np.array(inp.lidar).reshape(4, 72); lidar[2, 36] = 3.0      # a LiDAR hit the camera sees as free space
        inp2 = c.EvidenceInput.build(**{**{f: getattr(inp, f) for f in inp.__dataclass_fields__}, "lidar": lidar.reshape(-1)})
        m = om.LocalObstacleMap(); r = m.integrate(inp2, va.validate(inp2, q2a.CAM, q2a.LID))
        self.assertGreater(r.conflict_cells, 0)
        hit = np.array(q2a.LID.azimuth_rad())[36]
        p = (3.0 * math.cos(hit), 3.0 * math.sin(hit))
        self.assertIn(m.state_at(p, 0.0), (om.OCCUPIED, om.CONFLICT))
        self.assertNotEqual(m.segment_status((0, 0), (5.0 * math.cos(hit), 5.0 * math.sin(hit)), 0.1, 0.0)[0], om.SAFE)

    def test_ego_translation_and_rotation_keep_obstacles_in_place(self):
        bars = ((4.0, 0.0, 0.3),)
        m = om.LocalObstacleMap()
        for k, (xy, yaw) in enumerate((((0.0, 0.0), 0.0), ((1.0, 0.5), math.radians(30.0)), ((1.5, -0.5), math.radians(-20.0)))):
            inp, rep = frame_at(bars, pos_xy=xy, yaw=yaw, stamp=0.1 * k)
            self.assertTrue(m.integrate(inp, rep).accepted)
        occ = np.argwhere(np.isfinite(m.occ_s))
        world = m.origin_xy + (occ + 0.5) * m.p.cell_m
        self.assertTrue(len(world) > 0)
        self.assertLess(np.max(np.linalg.norm(world - np.array([4.0, 0.0]), axis=1)), 0.3 + 2 * m.p.cell_m)   # no ghosts
        self.assertEqual(m.state_at((3.75, 0.0), 0.2), om.OCCUPIED)

    def test_partial_field_of_view_without_lidar(self):
        inp, rep = frame_at((), lidar=np.full(288, np.nan))                 # LiDAR invalid: camera only (87 deg)
        m = om.LocalObstacleMap(); r = m.integrate(inp, va.validate(inp, q2a.CAM, q2a.LID))
        self.assertIn("LIDAR_NOT_INTEGRATED", r.reasons)
        self.assertEqual(m.state_at((3.0, 0.0), 0.0), om.FREE)
        ray = q2a.geo.native_camera_rays(q2a.CAM, q2a.PROF.obstacle_native_wh)[12, 20]    # a near-horizontal native ray
        origin = np.array(q2a.CAM.camera_offset)[:2]
        self.assertEqual(m.state_at(origin + 9.0 * ray[:2], 0.0), om.FREE)  # 10 m camera no-return: free up to it
        self.assertEqual(m.state_at(origin + 10.6 * ray[:2], 0.0), om.UNKNOWN)   # ... and never beyond the far plane
        self.assertEqual(m.state_at((9.5, 0.0), 0.0), om.UNKNOWN)          # between native rays (2.72 deg): unobserved
        self.assertEqual(m.state_at((0.0, 3.0), 0.0), om.UNKNOWN)          # outside the camera FOV
        self.assertEqual(m.state_at((-3.0, 0.0), 0.0), om.UNKNOWN)

    def test_late_observed_obstacle_overrides_old_free_space(self):
        m = om.LocalObstacleMap()
        inp, rep = frame_at((), stamp=0.0); m.integrate(inp, rep)
        self.assertEqual(m.segment_status((0, 0), (5, 0), 0.3, 0.0)[0], om.SAFE)
        inp, rep = frame_at(((4.0, 0.0, 0.3),), stamp=0.1); m.integrate(inp, rep)
        self.assertEqual(m.state_at((3.75, 0.0), 0.1), om.OCCUPIED)
        self.assertEqual(m.segment_status((0, 0), (5, 0), 0.3, 0.1)[0], om.BLOCKED)

    def test_frame_order_altitude_and_input_contract(self):
        m = om.LocalObstacleMap()
        inp, rep = frame_at((), stamp=0.2); self.assertTrue(m.integrate(inp, rep).accepted)
        inp, rep = frame_at((), stamp=0.1)
        self.assertEqual(m.integrate(inp, rep).reasons, ("FRAME_NOT_NEWER_THAN_MAP",))
        inp, rep = frame_at((), stamp=0.3, quat=np.array([.1, 0., 0., math.sqrt(.99)]))
        self.assertEqual(m.integrate(inp, rep).reasons, ("POSE_INVALID",))
        pos = np.array([0.0, 0.0, q2a.POS[2] + 0.5])
        clock = c.Clock(0.4, 0.4, "DECLARED_SIM_SCHEDULE")
        hi = q2a.make(render_target=False, position=pos, decision=0.4, cam_clock=clock, lidar_clock=clock)
        r = m.integrate(hi, va.validate(hi, q2a.CAM, q2a.LID))
        self.assertIn("ALTITUDE_CHANGED_MAP_RESET", r.reasons)

    def test_no_ground_truth_and_no_simulator_imports(self):
        src = (ROOT / "motar_eval/q02/obstacle_map.py").read_text()
        tree = ast.parse(src)
        mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        mods |= {(n.module or "").split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 0}
        self.assertFalse(mods & {"torch", "isaacgym", "aerial_gym", "warp", "random"})
        for word in ("target_position", "obstacle_positions", "env_asset", "ground_truth_obstacle"):
            self.assertNotIn(word, src)
        m, _ = built(((4.0, 0.0, 0.3),))
        self.assertFalse(m.validity(0.0)["ground_truth_used"])


if __name__ == "__main__":
    unittest.main()
