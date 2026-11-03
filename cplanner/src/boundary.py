"""P0-R input boundary: fixed-dtype, fixed-shape numeric arrays only.

Rule (replaces the v3-v7 object boundary of signed records, packets and candidate sets):
- an argument is accepted only if `type(a) is numpy.ndarray` (identity, so no subclass, no metaclass and no
  `__eq__` is consulted), its dtype is an instance of exactly the expected numpy dtype class in native byte order
  (so no object, structured, string, bool or reduced-precision dtype), and its shape is exactly the expected tuple;
- it is then copied ONCE into a C-ordered, read-only array owned by the core (`np.array(..., copy=True)` on an exact
  numeric ndarray runs no Python-level caller code), and the caller's array is never read again.

This validates the FORMAT of the data. It says nothing about whether a declared status, range, health or pose is
true: those stay declarations (health_basis DECLARED_NOT_AUTHENTICATED).
"""
import numpy as np

F64 = type(np.dtype(np.float64))
I64 = type(np.dtype(np.int64))
I8 = type(np.dtype(np.int8))
_CANON = {F64: np.float64, I64: np.int64, I8: np.int8}


def exact_copy(a, dtype_cls, shape):
    """A private read-only copy of `a`, or None if `a` is not an exact ndarray of `dtype_cls` and `shape`."""
    if type(a) is not np.ndarray:
        return None
    dt = a.dtype
    if type(dt) is not dtype_cls or dt.isnative is not True:
        return None
    if a.shape != shape:
        return None
    out = np.array(a, dtype=_CANON[dtype_cls], copy=True, order="C")
    if type(out) is not np.ndarray or type(out.dtype) is not dtype_cls or not out.dtype.isnative or out.shape != shape:
        return None
    out.flags.writeable = False
    return out


def exact_paths(a, k_max):
    """Paths: an exact float64 ndarray of shape (K, 2, 2) with 1 <= K <= k_max, copied; else None."""
    if type(a) is not np.ndarray:
        return None
    dt = a.dtype
    if type(dt) is not F64 or dt.isnative is not True:
        return None
    shp = a.shape
    if len(shp) != 3 or shp[1:] != (2, 2) or not 1 <= shp[0] <= k_max:
        return None
    out = np.array(a, dtype=np.float64, copy=True, order="C")
    if type(out) is not np.ndarray or type(out.dtype) is not F64 or not out.dtype.isnative or out.shape != shp:
        return None
    out.flags.writeable = False
    return out
