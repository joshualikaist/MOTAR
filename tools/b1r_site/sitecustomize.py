"""B1-R opt-in start-up hook (docs/plans/prereg_b1r_classical_baseline_v1.md §3).

Python imports this module at start-up only when ``tools/b1r_site`` is on ``PYTHONPATH``, which
``tools/b1r_run.py`` arranges for B1-R cells alone. Without ``MOTAR_B1R_ARM`` it does nothing.
With it, it loads ``tools/b1r_hook.py`` by file path and registers the post-import patch. Only the
standard library is imported here (Isaac Gym must still be imported before torch), and an install
error ends the process, because ``site.py`` would otherwise swallow it and the cell would silently
run unpatched.
"""
import os

if os.environ.get("MOTAR_B1R_ARM", "").strip():
    try:
        import importlib.util as _util
        import sys as _sys

        _path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "b1r_hook.py")
        _spec = _util.spec_from_file_location("b1r_hook", _path)
        _hook = _util.module_from_spec(_spec)
        _sys.modules["b1r_hook"] = _hook
        _spec.loader.exec_module(_hook)
        _hook.install()
    except BaseException as _exc:  # fail closed: never run a B1-R cell unpatched
        try:
            os.write(2, ("b1r sitecustomize: install failed: %r\n" % (_exc,)).encode())
        finally:
            os._exit(3)
