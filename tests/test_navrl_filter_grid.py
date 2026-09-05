"""The grid runner must refuse anything but governor knobs, keep names unique, and resolve policies."""
import importlib.util, json, os, tempfile, unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
_S = importlib.util.spec_from_file_location("grid_test", ROOT / "tools/run_navrl_filter_grid.py")
G = importlib.util.module_from_spec(_S); _S.loader.exec_module(G)


class SpecValidation(unittest.TestCase):
    def good(self):
        return {"seed": 523, "cells": [{"name": "a", "policy": "T0", "bars": 70, "env": {"NAVRL_SPEED_GOVERNOR": "stopcap"}}]}

    def test_good_spec_passes(self):
        G.validate_spec(self.good())

    def test_non_governor_env_refused(self):
        s = self.good(); s["cells"][0]["env"]["NAVRL_V2_GOAL_DIST_MIN"] = "6"
        with self.assertRaisesRegex(SystemExit, "only the governor may vary"):
            G.validate_spec(s)

    def test_duplicate_and_bad_names_and_bars(self):
        s = self.good(); s["cells"].append(dict(s["cells"][0]))
        with self.assertRaisesRegex(SystemExit, "duplicate"):
            G.validate_spec(s)
        s = self.good(); s["cells"][0]["name"] = "bad name/with slash"
        with self.assertRaisesRegex(SystemExit, "bad cell name"):
            G.validate_spec(s)
        s = self.good(); s["cells"][0]["bars"] = 5
        with self.assertRaisesRegex(SystemExit, "bars out of range"):
            G.validate_spec(s)

    def test_shipped_specs_validate(self):
        for name in ("grid_d1_density_filter_T0.json", "grid_l1_halfwidth_T0.json", "grid_d3_lineage_ep25000.json"):
            spec = G.validate_spec(json.loads((ROOT / "docs/specs" / name).read_text()))
            self.assertGreaterEqual(len(spec["cells"]), 4, name)
        d1 = json.loads((ROOT / "docs/specs/grid_d1_density_filter_T0.json").read_text())
        self.assertEqual(len(d1["cells"]), 20)
        self.assertEqual(sorted({c["bars"] for c in d1["cells"]}), [70, 100, 130, 160, 205])


class CellEnv(unittest.TestCase):
    def test_cell_env_pins_density_records_and_overrides_only_the_knob(self):
        class Env:
            def evaluation_env(self, *a, **k):
                return {"NAVRL_V2_DENSITIES": "70", "NAVRL_SPEED_GOVERNOR": "off", "NAVRL_STAR_CONVEX_SHADOW": "1"}
        spec = {"seed": 523, "frame_sample_every": 50}
        cell = {"name": "x", "policy": "T0", "bars": 205, "env": {"NAVRL_SPEED_GOVERNOR": "stopcap", "NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M": "0.8"}}
        env = G.cell_env(Env(), spec, cell, Path("/tmp/r"), Path("/tmp/r/x"))
        self.assertEqual(env["NAVRL_V2_DENSITIES"], "205")
        self.assertEqual(env["NAVRL_SPEED_GOVERNOR"], "stopcap")
        self.assertEqual(env["NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M"], "0.8")
        self.assertEqual(env["NAVRL_SPEED_GOVERNOR_BRAKE_MPS2"], "2.0")
        self.assertEqual(env["NAVRL_CG_FRAME_SAMPLE_EVERY"], "50")
        self.assertEqual(env["NAVRL_CONTACT_GEOMETRY"], "1")
        self.assertEqual(env["NAVRL_STAR_CONVEX_SHADOW"], "0")


if __name__ == "__main__":
    unittest.main()
