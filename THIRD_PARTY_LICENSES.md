# Third-party materials

What this repository contains that it did not write, where each piece came from, and whether it
is redistributed here. Code licences and dataset licences are listed separately, because they are
different instruments and conflating them is how a redistribution problem gets missed.

Status labels are deliberately narrow:

| Label | Meaning |
|---|---|
| `VERIFIED` | The licence was read from the upstream source or from a file in this repository, and the use here matches it. |
| `NEEDS_CONFIRMATION` | The licence is known but whether this use is permitted has not been confirmed. **Treat as a release blocker.** |
| `NOT_REDISTRIBUTED` | Nothing from this component is in the repository; it is downloaded or installed separately. |

**This file is not legal advice.** It records what was checked and what was not.

---

## Code

| Component | Source | Licence | How it is used | Redistributed here | Path | Status |
|---|---|---|---|---|---|---|
| Aerial Gym Simulator | `github.com/ntnu-arl/aerial_gym_simulator` | BSD-3-Clause | This repository is a fork. Simulator, controllers, sensors and examples derive from it. | Yes, extensively | `aerial_gym/`, `resources/`, root `LICENSE` | `VERIFIED` |
| rl_games | `github.com/Denys88/rl_games` | MIT | PPO implementation and runner, imported as a dependency | No, installed | — | `NOT_REDISTRIBUTED` |
| NavRL | `github.com/Zhefan-Xu/NavRL` | see upstream | Ideas adapted: observation structure and navigation formulation. No source copied. | No | — | `NEEDS_CONFIRMATION` — upstream licence not recorded here |
| NVIDIA Isaac Gym (Preview 4) | NVIDIA developer download | NVIDIA licence, manual download | Physics and rendering backend | No, manual install | — | `NOT_REDISTRIBUTED` |
| NVIDIA Warp | `warp-lang` on PyPI | Apache-2.0 | Ray-cast camera and LiDAR kernels | No, installed | — | `NOT_REDISTRIBUTED` |
| urdfpy | `github.com/mmatl/urdfpy` | MIT | URDF parsing for the renderer asset loader | No, installed (a local source checkout in this environment) | — | `NOT_REDISTRIBUTED` |
| trimesh | `github.com/mikedh/trimesh` | MIT | Mesh geometry behind urdfpy and the asset loader | No, installed | — | `NOT_REDISTRIBUTED` |
| YOLOv5 (`ultralytics/yolov5`) | `github.com/ultralytics/yolov5` | **AGPL-3.0** | Detector training and evaluation for the real-imagery track | **No source vendored** — verified: zero tracked files match `yolov5` | — | `VERIFIED` |

**On AGPL.** Running YOLOv5 to produce measurements does not place this repository under AGPL.
Publishing code built *on top of* that source would. Nothing from it is vendored here, and that
is a constraint on future architecture choices rather than a current problem.

**On the root `LICENSE`.** It is the upstream BSD-3-Clause notice, copyright *Autonomous Robots
Lab, NTNU*. Contributions made in this fork carry no separate copyright line.
`NEEDS_CONFIRMATION`: decide whether to add one before public release.

---

## Datasets and derived artefacts

| Component | Source | Licence | How it is used | Redistributed here | Path | Status |
|---|---|---|---|---|---|---|
| ETH `drone-tracking-datasets` ds5 | ETH Zurich | **CC BY-NC-SA 4.0** | cam0 video and survey-grade position ground truth, for the size-to-range measurement | **Yes — derived frames**, see below | `results/eth_ds5_intake_2026-09-10/` | **`NEEDS_CONFIRMATION`** |
| NPS-Drones | Naval Postgraduate School | see upstream terms | Detector training set | No — raw data under untracked `datasets/` | — | `NEEDS_CONFIRMATION` — terms not recorded here |
| Det-Fly | Det-Fly authors | see upstream terms | Zero-shot and joint-training evaluation | No — raw data untracked | — | `NEEDS_CONFIRMATION` — terms not recorded here |
| Model weights (`artifacts/*.pth`) | Produced by this project | follows this repository | Detector checkpoints v1–v7, ~100 KB total | Yes | `artifacts/` | `VERIFIED` |
| Example weights (`aerial_gym/examples/**/*.pth`, `aerial_gym/utils/vae/weights/`) | Aerial Gym upstream | BSD-3-Clause with the fork | Upstream example policies and a VAE, ~50 MB, 11 files | Yes, inherited from upstream | as listed | `VERIFIED` |

### The ETH-derived frames are the open item

`results/eth_ds5_intake_2026-09-10/` contains **34 tracked JPEG images** and a 1.3 MB review
archive (`eth_ds5_human_review_pack.zip`, 14 entries) built from ds5 cam0 video frames. These are
derivative works of a **CC BY-NC-SA 4.0** dataset, and that licence carries two conditions the
repository's own BSD-3-Clause notice does not satisfy:

- **ShareAlike** — a derivative must be offered under the same licence, not under BSD-3-Clause.
- **NonCommercial** — the dataset restricts commercial use; BSD-3-Clause does not.

So the root `LICENSE` must not be read as covering these files. Before public release, one of:

1. keep them and label them explicitly as CC BY-NC-SA 4.0 with attribution to ETH Zurich, stating
   that the repository licence does not apply to that directory; or
2. remove the derived frames from tracking and keep only the receipts, which already carry the
   measurements and hashes; or
3. confirm with the dataset authors that this use is permitted.

`results/eth_ds5_intake_2026-09-10/THIRD_PARTY_NOTICE.md` carries this notice next to the files,
so the attribution travels with them regardless of which option is chosen.

---

## Assets and figures

| Component | Source | Licence | Redistributed | Path | Status |
|---|---|---|---|---|---|
| Diagrams and presentation figures | Produced by this project | follows this repository | Yes | `docs/assets/` | `VERIFIED` |
| URDF and mesh assets | Aerial Gym upstream plus assets generated here | BSD-3-Clause / this repository | Yes | `resources/` | `VERIFIED` |

---

## What was checked, and how

- Secrets and credentials: a pattern scan over tracked text found **none**.
- Vendored YOLOv5: **zero** tracked paths match, so the AGPL boundary is not crossed here.
- Raw datasets: **zero** tracked files under `datasets/`; that directory is gitignored.
- Weights: 18 tracked `.pth`, split 11 upstream examples (~50 MB) and 7 project artifacts (~100 KB).

**Not checked:** git history for material removed from the working tree, and the upstream terms of
NPS-Drones, Det-Fly and NavRL, none of which are recorded in this repository.
