"""tools/sitl_bridge.py Policy: the checkpoint's action-distribution schema is exported before the player is built.

Static check (the bridge stubs aerial_gym packages in sys.modules, so it is not imported in the shared PUBLIC_CPU
process). Without the export the CPU player used NAVRL_ACTION_MU_SCALE=1 instead of Policy F's 1.0,0.4,1.0,1.0 (S3a on
G8 r2, 2026-11-05: lateral action error 0.388). The functional check is S3a itself.
"""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def calls_in_order(func):
    names = []
    for node in ast.walk(func):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            names.append((node.lineno, node.col_offset, node.func.attr))
    return [n for _, _, n in sorted(names)]


class BridgePolicyContractTest(unittest.TestCase):
    def setUp(self):
        tree = ast.parse((ROOT / "tools/sitl_bridge.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Policy")
        self.init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")

    def test_schema_exported_before_player_is_built(self):
        names = calls_in_order(self.init)
        self.assertIn("build_player", names)                     # positive: the player is built here
        self.assertIn("apply_schema_environment", names)
        self.assertLess(names.index("apply_schema_environment"), names.index("build_player"))

    def test_order_check_rejects_the_old_code(self):
        old = ast.parse("def __init__(self):\n    schema = a.read_checkpoint_schema(s)\n"
                        "    self.player = a.build_player(c, schema, 'cpu', t)\n").body[0]
        self.assertNotIn("apply_schema_environment", calls_in_order(old))
        self.assertIn("build_player", calls_in_order(old))


if __name__ == "__main__":
    unittest.main()
