import sys
from pathlib import Path
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
sys.path.insert(0, str(ROOT / "docs/evidence/q02b_q03_cpu_package_2026-10-22"))
import shadow_benchmark as sb
script, labels = sb.scenarios()["S9_occlusion_growing_covariance"]
rows = sb.run("H2", "S9", script, labels)
for r in rows[5:]:
    print(r.decision_index, r.decision, r.hypothesis, r.identity_verdict, r.plan_status, r.in_loss, r.warranted, r.reasons)
