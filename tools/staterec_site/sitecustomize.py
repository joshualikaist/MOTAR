"""G8 opt-in start-up hook (docs/plans/prereg_sitl_reference_g8g9_v1_amendment1_2026-11-05.md).

Python imports this module at start-up only when ``tools/staterec_site`` is on ``PYTHONPATH``, which
``tools/sitl_g8_ref_dump.py`` arranges for G8 cells alone. Without ``MOTAR_STATE_RECORD`` it does nothing. With it, it
loads ``tools/staterec_hook.py`` by file path and registers the post-import patch. Only the standard library is
imported here, and an install error ends the process (fail closed).
"""
import os

if os.environ.get("MOTAR_STATE_RECORD", "").strip():
    try:
        import importlib.util as _util
        import sys as _sys

        _path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "staterec_hook.py")
        _spec = _util.spec_from_file_location("staterec_hook", _path)
        _hook = _util.module_from_spec(_spec)
        _sys.modules["staterec_hook"] = _hook
        _spec.loader.exec_module(_hook)
        _hook.install()
    except BaseException as _exc:
        try:
            os.write(2, ("staterec sitecustomize: install failed: %r\n" % (_exc,)).encode())
        finally:
            os._exit(3)
