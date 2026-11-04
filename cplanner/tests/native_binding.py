"""SHA-256 binding of the native coverage library (M3 review I3), with positive and negative controls. Not a gate.

Builds cplanner/native/coverage_b.c with native/build.py into WORK_DIR (never inside the repository), then runs each
case in a fresh process, because coverage_native caches the loaded library:
  rebuild      the rebuilt library has the qualified SHA-256 (same source, same compiler)       positive control
  qualified    it loads, reports that hash, and returns the verbatim _coverage_B result on a
               certified and an uncertified case (so the native code really ran)               positive control
  tampered     a copy with one byte changed is refused: load() None, a reason is given, and
               coverage_B_native returns the fail-closed NATIVE_COVERAGE_UNAVAILABLE             negative control
  missing      a path with no file is refused the same way                                       negative control
  declared     the tampered copy loads only when its own hash is declared in
               CPLANNER_COVERAGE_SO_SHA256 (an explicit candidate), and a wrong declared hash
               refuses the qualified library                                                     both
usage: native_binding.py WORK_DIR OUT_JSON
"""
import json
import os
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
CP = HERE.parent
QUALIFIED = "afbd3e68df0743a872eb4b142565bf3d69860fe83ef0b303dc98728c9b45a5aa"

CHILD = r'''
import json, math, sys
sys.path.insert(0, sys.argv[1])
import coverage_b as CB, coverage_native as CN, registry as REG
cfg = REG.CONFIGS["MOTAR-72x4-RUNTIME"]
R, res, budget = cfg.plan_radius, cfg.resolution, cfg.max_nodes
def rays(n, reach):
    return [((0.0, 0.0), (reach * math.cos(2 * math.pi * k / n), reach * math.sin(2 * math.pi * k / n))) for k in range(n)]
cases = {"dense": (((0.0, 0.0), (0.6, 0.0)), rays(720, 8.0), []),
         "sparse": (((0.0, 0.0), (0.6, 0.0)), rays(4, 8.0), [])}
lib = CN.load()
out = {"loaded": lib is not None, "problem": CN.load_problem(), "loaded_sha256": CN.loaded_sha256(),
       "expected_sha256": CN.expected_sha256(), "cases": {}}
for k, (seg, free, surf) in cases.items():
    py = CB._coverage_B(seg, free, surf, REG.BAR_HALF_SIZES, R, res, budget)
    nat = CN.coverage_B_native(seg, free, surf, REG.BAR_HALF_SIZES, R, res, budget)
    out["cases"][k] = {"py": py, "native": nat, "equal": json.dumps(py, sort_keys=True) == json.dumps(nat, sort_keys=True)}
print(json.dumps(out, default=str))
'''


def child(so, declared=None):
    env = {"PATH": "/usr/bin:/bin", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1", "CPLANNER_COVERAGE_SO": str(so)}
    if declared is not None:
        env["CPLANNER_COVERAGE_SO_SHA256"] = declared
    r = subprocess.run([sys.executable, "-I", "-B", "-c", CHILD, str(CP / "src")], env=env, capture_output=True, text=True)
    if r.returncode:
        return {"error": r.stderr[-1500:]}
    return json.loads(r.stdout)


def refused(c):
    return (not c.get("loaded") and bool(c.get("problem"))
            and all(v["native"] == {"certified": False, "reason": "NATIVE_COVERAGE_UNAVAILABLE", "nodes": 0}
                    for v in c["cases"].values()))


def main():
    work, out = pathlib.Path(sys.argv[1]).resolve(), pathlib.Path(sys.argv[2])
    if work.exists() or out.exists():
        raise SystemExit("refusing to overwrite")
    work.mkdir(parents=True)
    so = work / "libcoverage_b.so"
    b = subprocess.run([sys.executable, "-I", "-B", str(CP / "native" / "build.py"), str(so)], capture_output=True, text=True)
    build = json.loads(b.stdout) if b.returncode == 0 else {"error": b.stderr[-1500:]}
    bad = work / "libcoverage_b_tampered.so"
    data = bytearray(so.read_bytes())
    data[-1] ^= 0xFF                                   # last byte: still a loadable ELF; only the hash changes
    bad.write_bytes(bytes(data))
    import hashlib
    bad_sha = hashlib.sha256(bytes(data)).hexdigest()
    res = {"build": build,
           "qualified": child(so),
           "tampered": child(bad),
           "missing": child(work / "no_such_library.so"),
           "declared_tampered": child(bad, declared=bad_sha),
           "declared_wrong": child(so, declared="0" * 64)}
    q, dt = res["qualified"], res["declared_tampered"]
    checks = {
        "rebuild_has_qualified_sha": build.get("so_sha256") == QUALIFIED,
        "qualified_loads": q.get("loaded") is True and q.get("loaded_sha256") == QUALIFIED,
        "qualified_equals_verbatim": bool(q.get("cases")) and all(v["equal"] for v in q["cases"].values()),
        "qualified_cases_nonvacuous": bool(q.get("cases")) and q["cases"]["dense"]["py"]["certified"] is True
                                       and q["cases"]["sparse"]["py"]["certified"] is False,
        "tampered_refused": refused(res["tampered"]),
        "missing_refused": refused(res["missing"]),
        "declared_candidate_loads": dt.get("loaded") is True and dt.get("loaded_sha256") == bad_sha,
        "declared_wrong_refuses_qualified": refused(res["declared_wrong"]),
    }
    res["checks"], res["pass"] = checks, all(checks.values())
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=str)
        fh.write("\n")
    print(json.dumps({"checks": checks, "pass": res["pass"]}, indent=1))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
