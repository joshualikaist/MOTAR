"""ctypes wrapper of cplanner/native/libcoverage_b.so: `coverage_B_native` has the signature and the result
dictionary of `coverage_b._coverage_B` (M3 candidate; NOT used by any qualified configuration until its equivalence
gate passes; tests/m3_native_equivalence.py compares whole result dictionaries with the verbatim Python).

The library path is `CPLANNER_COVERAGE_SO` or `../native/libcoverage_b.so`. `load()` returns None if the library is
missing or unloadable; callers must then refuse (fail closed), never fall back silently.
"""
import ctypes
import os
import pathlib

import numpy as np

_SO = pathlib.Path(os.environ.get("CPLANNER_COVERAGE_SO",
                                  pathlib.Path(__file__).resolve().parents[1] / "native" / "libcoverage_b.so"))
_lib = None
_D, _L, _P = ctypes.c_double, ctypes.c_long, ctypes.POINTER(ctypes.c_double)
REASONS = {0: "ALL_COLLIDING_CENTER_CELLS_EXCLUDED", 1: "OBSERVED_SURFACE_INTERSECTION", 2: "NO_VALID_RAYS",
           3: "NODE_BUDGET_UNKNOWN", 4: "UNRESOLVED_CENTER_CELL", -1: "NATIVE_STACK_LIMIT"}


def load():
    global _lib
    if _lib is None:
        try:
            lib = ctypes.CDLL(str(_SO))
            f = lib.coverage_b
            f.restype = ctypes.c_int
            f.argtypes = [_D, _D, _D, _D, _P, _L, _P, _L, _P, _L, _D, _D, _L, ctypes.POINTER(_L), _P, _P]
            for name, n in (("cov_py_hypot", 2), ("cov_point_segment", 6), ("cov_segment_box_distance", 8)):
                g = getattr(lib, name)
                g.restype, g.argtypes = _D, [_D] * n
            lib.cov_crosses.restype = ctypes.c_int
            lib.cov_crosses.argtypes = [_D] * 8 + [ctypes.c_int]
            _lib = lib
        except (OSError, AttributeError):
            return None
    return _lib


def path():
    return str(_SO)


def _arr(x, cols):
    a = np.ascontiguousarray(np.asarray(x, dtype=np.float64).reshape(-1, cols))
    return a, a.ctypes.data_as(_P)


def coverage_B_native(path_, free, surfaces, shapes, radius, resolution, max_nodes):
    lib = load()
    if lib is None:
        return {'certified': False, 'reason': 'NATIVE_COVERAGE_UNAVAILABLE', 'nodes': 0}
    (ax, ay), (bx, by) = path_
    fr, fp = _arr([(s[0], s[1], e[0], e[1]) for s, e in free] if len(free) else np.zeros((0, 4)), 4)
    su, sp = _arr(list(surfaces) if len(surfaces) else np.zeros((0, 2)), 2)
    sh, hp = _arr(list(shapes), 2)
    nodes = _L(0)
    shape = (_D * 2)()
    cell = (_D * 4)()
    code = lib.coverage_b(float(ax), float(ay), float(bx), float(by), fp, len(fr), sp, len(su), hp, len(sh),
                          float(radius), float(resolution), int(max_nodes), ctypes.byref(nodes),
                          ctypes.cast(shape, _P), ctypes.cast(cell, _P))
    reason = REASONS.get(code, "NATIVE_UNKNOWN_CODE")
    if code == 0:
        return {'certified': True, 'reason': reason, 'nodes': nodes.value}
    if code in (1, 2):
        return {'certified': False, 'reason': reason, 'nodes': 0}
    if code == 4:
        return {'certified': False, 'reason': reason, 'nodes': nodes.value,
                'shape_halfsize': [float(shape[0]), float(shape[1])], 'cell': [float(c) for c in cell]}
    return {'certified': False, 'reason': reason, 'nodes': nodes.value}
