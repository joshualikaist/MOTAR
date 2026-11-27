import sys
from pathlib import Path
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q02b_timing as t
from motar_eval.q02 import shadow as sh
from motar_eval.q02.history import loss_state
SEEN = dict(detected=True)
UNV = dict(detected=True, age=5.0)   # flagged, identity NOT_TESTABLE (as in the repo's own test)
for policy in ("H1", "H3", "H2"):
    rec = t.ShadowRecorder(policy); rec.reset(1)
    for k, kw in enumerate([SEEN]*4 + [UNV]*14):
        d, c = rec.observe(1, k, t.frame(k, **kw), t.ORACLE_PROXY)
        e = rec.history.entries[-1]
        ls = loss_state(rec.history)
        if k >= 3 and policy == "H1":
            print(k, "flag", e.detected, "verdict", e.identity_verdict, "unverified", e.identity_unverified, "informative", e.informative,
                  "| lost", ls.currently_lost, "events", ls.recent_loss_events, "| state", d.state_after, d.decision, d.reasons[:3])
    print(policy, "last:", d.decision, d.reasons)
