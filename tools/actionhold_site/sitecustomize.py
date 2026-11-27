"""G7 opt-in start-up hook (docs/plans/prereg_actionhold_g7_v1.md).

Python imports this module at start-up only when ``tools/actionhold_site`` is on ``PYTHONPATH``, which
``tools/actionhold_run.py`` arranges for G7 cells alone. Without ``MOTAR_ACTION_HOLD_K`` / ``MOTAR_ACTION_DELAY_D`` it
does nothing. With either, it loads ``tools/actionhold_hook.py`` by file path and registers the post-import patch.
Only the standard library is imported here (Isaac Gym must still be imported before torch), and an install error ends
the process, because ``site.py`` would otherwise swallow it and the cell would silently run unpatched.
"""
import os

if os.environ.get("MOTAR_ACTION_HOLD_K", "").strip() or os.environ.get("MOTAR_ACTION_DELAY_D", "").strip():
    try:
        import importlib.util as _util
        import sys as _sys

        _path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "actionhold_hook.py")
        _spec = _util.spec_from_file_location("actionhold_hook", _path)
        _hook = _util.module_from_spec(_spec)
        _sys.modules["actionhold_hook"] = _hook
        _spec.loader.exec_module(_hook)
        _hook.install()
    except BaseException as _exc:  # fail closed: never run a G7 cell unpatched
        try:
            os.write(2, ("actionhold sitecustomize: install failed: %r\n" % (_exc,)).encode())
        finally:
            os._exit(3)
