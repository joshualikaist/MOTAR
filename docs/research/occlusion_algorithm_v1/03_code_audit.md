# 03 — Official source-code audit

**Method.**
- Shallow, no-checkout clones were made on 2026-10-19, reading blobs only with `git show`.
- Nothing was built, installed or run, so no repository is CODE_VERIFIED.
- Audit levels are README_ONLY, STRUCTURE_MAPPED or FUNCTIONS_READ.
- Clones were deleted after the audit; the commit SHAs below allow exact re-inspection.
- Detail and file:line maps are in [K1](notes/K1_code_audit.md) and [K2](notes/K2_code_audit.md).

## Repositories

| Repository | Commit (date) | Licence | Stack | Audit level | Target state in their simulation | Portability to MOTAR |
| --- | --- | --- | --- | --- | --- | --- |
| [Eva-Tracker](https://github.com/Yue-0/Eva-Tracker) | `636e26e` (2026-03-05) | **AGPL-3.0** | C++14, ROS1 Noetic | FUNCTIONS_READ | GT `/target/poses`, no occlusion gate | FoV-ESDF (`esdf.cpp:31,109,168`), in-frustum occlusion cost (`optim.cpp:345-379`), candidate correction (`plan.cpp:17`): **clean-room reimplementation only**. Optimiser: not applicable |
| [Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker) | `0a302a2` (2023-06-16) | **GPL-3.0** | C++14, ROS1 | FUNCTIONS_READ | GT odom → CV-EKF, no occlusion gate | Visible sector (`env.hpp:646`), angular visibility hinge (`traj_opt.cc:700`): clean-room only |
| [UAST](https://github.com/qinliangql/UAST) | `1fcf5e0` (2026-09-03) | **MIT** | Python 3.8, PyTorch 2.4.1 | FUNCTIONS_READ | GT odom → z-buffered visible-pixel mask | SDF line-of-sight term (`track_loss.py:62,114,141`) and z-buffer visibility (`test_uast_track.py:749-829`): **direct port possible** with notice. Search supervisor: reimplement. Network: not a replacement (no weights fetched, no dataset or training script) |
| [FASTER](https://github.com/mit-acl/faster) | `f431c97` (2025-03-10) | BSD-3 (needs Gurobi) | C++, ROS1 | FUNCTIONS_READ (limited) | n/a (no target) | Safe-backup idea only |
| [PANTHER](https://github.com/mit-acl/panther) | `50be45b` (2022-11-01) | BSD-3 (CasADi/IPOPT/HSL deps; solver files are LFS pointers) | C++, ROS1, MATLAB codegen | FUNCTIONS_READ (limited) | GT obstacle trajectories by default | Sigmoid FOV term (`main.m:285-320`): reimplement |
| BPOD | — | — | — | — | — | **No code link in the paper.** Ideas only |
| [OA-VAT](https://github.com/SHWplus/OA-VAT) | `cea1f6b` (2026-04-24) | README says MIT; **no LICENSE file**; vendored DINOv3 licence; YOLOE is AGPL | Python 3.10, torch 2.2.2, UnrealCV | FUNCTIONS_READ | Detector + re-ID + image KF | Confidence → noise mapping: reimplement. The confidence-aware KF methods in `kalman_filter.py` are **never called**; the loop uses pykalman instead |
| [YOPO](https://github.com/TJU-Aerial-Robotics/YOPO) (`YOPO-Simple` branch) | `bd8a7f9` (2026-08-14) | MIT | Python 3.8, torch 2.4.1, ROS1 | FUNCTIONS_READ | n/a (goal navigation) | 5×3 lattice (±36°/±20°, r = 5 m) + quintic (`policy/primitive.py:30-84`): **direct port possible** as a network-free candidate generator. Original paper now FULL_READ (G1-R); this code audit is YOPO-Simple `bd8a7f9`, not the paper-version `main`, which remains unaudited |
| [YOPO-Tracker](https://github.com/TJU-Aerial-Robotics/YOPO-Tracker) | `d313f86` (2026-10-15) | MIT | Python 3.12, ROS2 Jazzy, Unity | FUNCTIONS_READ | Own detection head + gated CV-KF; GT labels in training | KF + back-projection + costs: direct port possible. Brake-and-spin search (`test_yopo_ros.py:246`): reimplement |
| [NavRL](https://github.com/Zhefan-Xu/NavRL) | `3725bcc` (2025-07-03); local `reference/NavRL` identical | MIT | Python 3.10, Isaac Sim, torchrl | FUNCTIONS_READ | Known goal; dynamic obstacles from a detector | ORCA shield (`safeAction.cpp:53,139`): reimplement. **No NavRL++ code is released** |
| [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) | `bfda512` (2025-03-08) | **GPL-3.0** | C++14, ROS1 | STRUCTURE_MAPPED | n/a | Ideas only |

## Licence gate for MOTAR code

- **Copy with notice allowed:** UAST, YOPO and YOPO-Tracker (MIT); FASTER and PANTHER (BSD-3). Each
  dependency still needs its own check.
- **Clean-room only:**
  - Eva-Tracker (AGPL-3.0), Elastic-Tracker and EGO-Planner (GPL-3.0). Re-derive from the paper; do
    not translate the code.
  - OA-VAT, because it has no LICENSE file, plus its vendored DINOv3 and AGPL YOLOE.
- **The BOIA v1 design copies no third-party code.** Every module is specified from the papers'
  equations and is reimplemented in Python. The YOPO lattice and the UAST line-of-sight sampling may
  be ported later, with an MIT notice, if that is simpler.

## What no repository provides

- **No GT-independent target-occlusion signal.** Eva and Elastic feed the GT target state without
  occlusion gating; UAST gates GT through a depth z-buffer.
- **No arbitration over a frozen learned policy.**
- **No capture or terminal phase.** All are keep-distance trackers or navigators.

That gap is what [`06_novelty_matrix.md`](06_novelty_matrix.md) examines.

## G1-R source-scope amendment

No new repository was cloned or executed in G1-R. The original YOPO paper's−J/high-score convention
does not match the audited YOPO-Simple cost/argmin convention; K2 remains a branch-specific code
audit, not a verification of paper-version main. All URLs for the four extras in
[P6](notes/P6_extra_papers.md) are **audit not done**. Their paper descriptions, including PRIMER's
runtime reject/reuse behavior, do not certify source implementation or weights. The old section
“What no repository provides” refers only to the eleven audited snapshots and cannot support a
general novelty claim beyond that scope.
