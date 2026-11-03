"""R2 rules 1 and 2: (a) the coverage core is AST-identical to v3-v7; (b) the bar catalog equals the snapshot and the
read-only bars_h3 URDFs (rpy 0); (c) the MOTAR manifest equals the warp formula; (d) static non-interference: the
certificate path and the evidence map never reference the label layer or a camera. usage: check_static.py OUT_JSON"""
import ast
import json
import pathlib
import re
import sys

import numpy as np

from harness import W, beam_dirs, dump
import registry as REG

V7 = pathlib.Path("/home/fair/motar_tmp/p0_v7_20261029")
NAMES = ("_coverage_B", "point_segment", "crosses", "segment_box_distance")
SRC = W / "src"


def fdump(path, names=NAMES):
    tree = ast.parse(pathlib.Path(path).read_text())
    return {n.name: ast.dump(n, include_attributes=False) for n in tree.body if isinstance(n, ast.FunctionDef)
            and n.name in names}


out = {}
ref = fdump(SRC / "coverage_b.py")
srcs = {"v3_run4": V7 / "baseline/v3_run4/certify_b.py", "v4_run5": V7 / "baseline/v4_run5/certify_b.py",
        "v5_run6": V7 / "baseline/v5_run6/certify_b.py", "v6_run7": V7 / "baseline/v6_run7/certify_b.py",
        "v7_run8": V7 / "src/certify_b.py"}
out["coverage_ast"] = {v: {k: fdump(p)[k] == ref[k] for k in NAMES} for v, p in srcs.items()}
out["coverage_ast_identical"] = all(all(x.values()) for x in out["coverage_ast"].values())

URDF = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/resources/models/environment_assets/bars_h3")
urdf = []
for i in range(40):
    t = (URDF / ("bar_%03d.urdf" % i)).read_text()
    sizes = {tuple(float(x) for x in m.split()) for m in re.findall(r'<box size="([^"]+)"', t)}
    rpys = set(re.findall(r'rpy="([^"]+)"', t))
    urdf.append((sizes, rpys))
snap = json.load(open(W / "snapshot/geometry_results.json"))["bars"]
out["catalog"] = {
    "n": len(REG.BAR_HALF_SIZES),
    "equals_snapshot": [tuple(b["size_m"][k] / 2 for k in (0, 1)) for b in snap] == list(REG.BAR_HALF_SIZES),
    "equals_urdf": all(len(s) == 1 and tuple(v / 2 for v in list(s)[0][:2]) == REG.BAR_HALF_SIZES[i]
                       and list(s)[0][2] == 3.0 for i, (s, _) in enumerate(urdf)),
    "urdf_rpy_all_zero": all(r == {"0 0 0"} for _, r in urdf),
    "bar_z_extent": REG.BAR_Z_EXTENT}
out["catalog_ok"] = out["catalog"]["equals_snapshot"] and out["catalog"]["equals_urdf"] and out["catalog"][
    "urdf_rpy_all_zero"]
diff = {lay: float(np.abs(REG.directions(lay) - beam_dirs(lay)).max()) for lay in ("P0R-72x1", "MOTAR-72x4")}
out["manifest_max_abs_diff"] = diff
out["manifest_ok"] = all(v < 1e-12 for v in diff.values())

FORBIDDEN = {"_labels", "labels", "camera", "cam", "TargetLabelLayer", "label_step", "label_t", "label_report"}


def names_in(node):
    s = set()
    for n in ast.walk(node):
        if isinstance(n, ast.Name):
            s.add(n.id)
        elif isinstance(n, ast.Attribute):
            s.add(n.attr)
        elif isinstance(n, ast.arg):
            s.add(n.arg)
    return s


core_tree = ast.parse((SRC / "core.py").read_text())
cls = [n for n in core_tree.body if isinstance(n, ast.ClassDef) and n.name == "CertifierCore"][0]
meths = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
cert_refs = (names_in(meths["_certify"]) | names_in(meths["certify"]) | names_in(meths["_result"])
             | names_in(meths["_certify_one"]))
em_tree = ast.parse((SRC / "evidence_map.py").read_text())
em_refs = names_in(em_tree)
imports = {}
for f in ("evidence_map.py", "coverage_b.py", "boundary.py", "registry.py"):
    t = ast.parse((SRC / f).read_text())
    imports[f] = sorted({a.name for n in ast.walk(t) if isinstance(n, ast.Import) for a in n.names}
                        | {n.module for n in ast.walk(t) if isinstance(n, ast.ImportFrom)})
out["noninterference_static"] = {
    "certify_path_forbidden_refs": sorted(cert_refs & FORBIDDEN),
    "evidence_map_forbidden_refs": sorted(em_refs & FORBIDDEN),
    "imports": imports,
    "map_and_core_modules_do_not_import_labels": all("labels" not in v and "core" not in v for v in imports.values()),
}
out["noninterference_static_ok"] = (not out["noninterference_static"]["certify_path_forbidden_refs"]
                                    and not out["noninterference_static"]["evidence_map_forbidden_refs"]
                                    and out["noninterference_static"]["map_and_core_modules_do_not_import_labels"])
out["pass"] = out["coverage_ast_identical"] and out["catalog_ok"] and out["manifest_ok"] and out[
    "noninterference_static_ok"]
dump(sys.argv[1], out)
print(json.dumps({k: v for k, v in out.items() if k != "coverage_ast"}, indent=1, default=str))
