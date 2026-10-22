import sys
from pathlib import Path
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q02b_timing as t
from motar_eval.q02.history import loss_state
SEEN, OCC = dict(detected=True), dict(bars=((4.0, 0.0, 0.3),))
UNV = dict(detected=True, age=5.0)
for name, script in (("unverified-then-occluded", [SEEN]*4 + [UNV]*10 + [OCC]*3), ("control: seen-then-occluded", [SEEN]*14 + [OCC]*3)):
    rec = t.ShadowRecorder("H1"); rec.reset(1)
    for k, kw in enumerate(script):
        d, _ = rec.observe(1, k, t.frame(k, **kw), t.ORACLE_PROXY)
        ls = loss_state(rec.history)
        if k >= 13:
            print(name, k, d.decision, "loss_duration", ls.loss_duration_s, "onset", ls.loss_onset_decision, d.reasons)
