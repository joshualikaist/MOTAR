"""G8 zero-action opt-in start-up hook (amendment 2). Inert unless MOTAR_ACTION_ZERO=1; fails closed on install errors."""
import os

if os.environ.get("MOTAR_ACTION_ZERO", "").strip():
    try:
        import importlib.util as _util
        import sys as _sys

        _path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "zeroaction_hook.py")
        _spec = _util.spec_from_file_location("zeroaction_hook", _path)
        _hook = _util.module_from_spec(_spec)
        _sys.modules["zeroaction_hook"] = _hook
        _spec.loader.exec_module(_hook)
        _hook.install()
    except BaseException as _exc:
        try:
            os.write(2, ("zeroaction sitecustomize: install failed: %r\n" % (_exc,)).encode())
        finally:
            os._exit(3)
