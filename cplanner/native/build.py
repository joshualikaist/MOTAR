"""Build cplanner/native/libcoverage_b.so from coverage_b.c with fixed flags; print a JSON build record.

IEEE double semantics are required for bit-identical results with coverage_b.py: no fast-math, no FMA contraction,
SSE2 doubles (x86-64 default; -mfpmath=sse is explicit), no -march (portable baseline ISA).
usage: build.py [OUT_SO]   (default: next to this file). Refuses to overwrite an existing library.
"""
import hashlib
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE / "coverage_b.c"
FLAGS = ["-O2", "-fPIC", "-shared", "-std=c99", "-ffp-contract=off", "-fno-fast-math", "-mfpmath=sse",
         "-fno-strict-aliasing", "-Wall", "-Werror"]


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def main():
    out = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else HERE / "libcoverage_b.so"
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    cmd = ["gcc"] + FLAGS + ["-o", str(out), str(SRC), "-lm"]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    ver = subprocess.run(["gcc", "--version"], stdout=subprocess.PIPE, universal_newlines=True).stdout.splitlines()[0]
    rec = {"cmd": cmd, "exit": r.returncode, "stderr": r.stderr[-2000:], "gcc": ver, "src_sha256": sha(SRC),
           "so_sha256": sha(out) if out.exists() else None, "so": str(out)}
    print(json.dumps(rec, indent=1))
    return r.returncode


if __name__ == "__main__":
    raise SystemExit(main())
