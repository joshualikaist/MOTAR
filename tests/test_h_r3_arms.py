"""CPU checks of the H-R3 scripted arms: geometry, switches, safety filter and input firewall."""
import ast
import inspect
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval import h_r3_arms as arms  # noqa: E402

ANGLES = torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72)
ELEV = torch.deg2rad(torch.tensor([20., 10., 0., -10.]))
Q = torch.tensor([[0., 0., 0., 1.]])


def scan_with_wall(x0, y_lo, y_hi):
    """Raw LiDAR [1,288] of a thin wall segment at x = x0, y in [y_lo, y_hi] (vertical, all rows)."""
    r = torch.full((4, 72), 12.0)
    for j, th in enumerate(ANGLES.tolist()):
        if math.cos(th) <= 0:
            continue
        y = x0 * math.tan(th)
        if y_lo <= y <= y_hi:
            for e, el in enumerate(ELEV.tolist()):
                r[e, j] = (x0 / math.cos(th)) / math.cos(el)
    return r.view(1, 288)


def act(arm, scan, belief=(6., 0., 1.)):
    return arms.scripted_action(arm, torch.tensor([[0., 0., 1.]]), Q, torch.tensor([belief]), scan, ANGLES, ELEV,
                                max_velocity=2.5, yaw_rate_max=3.0, lidar_far_m=12.0)


class GeometryTest(unittest.TestCase):
    def test_clear_line_of_sight_needs_no_offset(self):
        a, info = act("B3", torch.full((1, 288), 12.0))
        self.assertTrue(bool(info["clear_found"][0]))
        self.assertEqual(float(info["chosen_offset_m"][0]), 0.0)
        self.assertAlmostEqual(float(a[0, 0]), 1.0 / 2.5, places=5)
        self.assertAlmostEqual(float(a[0, 1]), 0.0, places=6)

    def test_occluder_produces_lateral_viewpoint_only_in_b3(self):
        scan = scan_with_wall(3.0, -0.3, 0.3)
        a3, info = act("B3", scan)
        self.assertTrue(bool(info["clear_found"][0]))
        self.assertGreater(abs(float(info["chosen_offset_m"][0])), 0.0)
        self.assertGreater(abs(float(a3[0, 1])), 0.1)
        for arm in ("B1", "B2"):
            a, info_b = act(arm, scan)
            self.assertAlmostEqual(float(a[0, 1]), 0.0, places=6)
            self.assertEqual(float(info_b["chosen_offset_m"][0]), float(info["chosen_offset_m"][0]))  # same compute

    def test_yaw_switch(self):
        scan = torch.full((1, 288), 12.0)
        b1, _ = act("B1", scan, belief=(4., 3., 1.))
        b2, _ = act("B2", scan, belief=(4., 3., 1.))
        self.assertEqual(float(b1[0, 3]), 0.0)
        self.assertGreater(float(b2[0, 3]), 0.0)  # belief to the left -> positive yaw rate
        self.assertAlmostEqual(float(b2[0, 3]), min(2.0 * math.atan2(3., 4.), 3.0) / 3.0, places=5)

    def test_safety_filter_removes_approach_component(self):
        scan = scan_with_wall(0.6, -1.0, 1.0)  # wall 0.6 m ahead
        a, _ = act("B1", scan)
        self.assertLess(float(a[0, 0]), 0.2 / 2.5)

    def test_belief_returns_do_not_block_sight(self):
        scan = scan_with_wall(5.9, -0.1, 0.1)  # the target's own LiDAR return at the belief
        _, info = act("B3", scan)
        self.assertEqual(float(info["chosen_offset_m"][0]), 0.0)


class FirewallTest(unittest.TestCase):
    def test_inputs_are_onboard_only(self):
        params = inspect.signature(arms.scripted_action).parameters
        forbidden = ("target_position", "gt", "semantic", "segmentation", "mask", "occlusion", "el0", "truth")
        for name in params:
            self.assertFalse(any(w in name.lower() for w in forbidden), name)
        tree = ast.parse((ROOT / "motar_eval/h_r3_arms.py").read_text())
        imports = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        self.assertEqual(imports, {"math", "torch"})


if __name__ == "__main__":
    unittest.main()
