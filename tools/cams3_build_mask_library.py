"""CAM-S3 (docs/plans/prereg_camonly_s3w_2026-11-27.md): build the measured invalid-depth mask library for training.

Input: the Jetson handheld recordings of 2026-11-26 (OAK-D Pro, colour-aligned depth 624 x 468, every frame's invalid
mask as np.packbits along rows; ~/motar_tmp/jetson_in_20261126/extract/motar_jetson_to_training_20261126_analysis/
<scene>/masks.npz, True = invalid). Output: one .npz with
  masks        bool [F, 234, 312]  the trained camera grid (312 x 234). A trained pixel is invalid only if all four
               source pixels of its 2 x 2 block are invalid: the runtime's min-valid projection keeps a trained pixel
               valid when any source return lands in it.
  scene_start  int64 [S + 1]      frame offsets of the scenes (frames of one scene are consecutive, in time order)
  scenes       str [S]
  source_sha256 str [S]           SHA-256 of each input masks.npz
Frames whose invalid fraction below the top band exceeds MAX_INVALID_BELOW_BAND are dropped (recorded): those are the
camera held against a near surface (below MinZ), which the simulator already produces from its own geometry; sampling
them onto arbitrary simulated scenes would blind frames for no physical reason.
usage: tools/cams3_build_mask_library.py SOURCE_DIR OUT_NPZ
"""
import hashlib
import json
import pathlib
import sys

import numpy as np

SCENES = ("044318_rotate", "050447_walk", "050939_wall", "051922_poles", "052033_floor", "052146_target")
SRC_W, SRC_H, OUT_W, OUT_H = 624, 468, 312, 234
TOP_BAND_ROWS = 10                    # 20 source rows: invalid in 100 % of frames of every scene
MAX_INVALID_BELOW_BAND = 0.5


def downsample_all_invalid(m):
    """[F, 468, 624] bool -> [F, 234, 312]: invalid iff the 2 x 2 source block is all invalid."""
    f = m.shape[0]
    return m.reshape(f, OUT_H, 2, OUT_W, 2).all(axis=(2, 4))


def load_scene(path):
    z = np.load(path)
    m = np.unpackbits(z["masks"], axis=-1)[..., :SRC_W].astype(bool)
    if m.shape[1:] != (SRC_H, SRC_W):
        raise ValueError("%s: masks %s, expected [F, %d, %d]" % (path, m.shape, SRC_H, SRC_W))
    stamps = np.asarray(z["stamps"])
    if stamps.shape[0] != m.shape[0] or not np.all(np.diff(stamps) > 0):
        raise ValueError("%s: stamps not strictly increasing per frame" % path)
    return m


def build(source_dir):
    source_dir = pathlib.Path(source_dir)
    out, starts, shas, report = [], [0], [], {}
    for scene in SCENES:
        path = source_dir / scene / "masks.npz"
        shas.append(hashlib.sha256(path.read_bytes()).hexdigest())
        small = downsample_all_invalid(load_scene(path))
        below = small[:, TOP_BAND_ROWS:, :].mean(axis=(1, 2))
        keep = below <= MAX_INVALID_BELOW_BAND
        kept = small[keep]
        out.append(kept)
        starts.append(starts[-1] + int(kept.shape[0]))
        report[scene] = {"frames": int(small.shape[0]), "kept": int(kept.shape[0]),
                         "invalid_below_band_median_kept": float(np.median(below[keep])) if keep.any() else None,
                         "invalid_below_band_p95_kept": float(np.percentile(below[keep], 95)) if keep.any() else None,
                         "top_band_invalid_fraction": float(small[:, :TOP_BAND_ROWS, :].mean())}
    masks = np.concatenate(out, axis=0)
    return masks, np.asarray(starts, np.int64), shas, report


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) != 2:
        raise SystemExit(__doc__.split("usage:")[1])
    out = pathlib.Path(argv[1])
    if out.exists():
        raise SystemExit("refusing to overwrite %s" % out)
    masks, starts, shas, report = build(argv[0])
    np.savez_compressed(out, masks=masks, scene_start=starts, scenes=np.asarray(SCENES), source_sha256=np.asarray(shas))
    summary = {"out": str(out), "sha256": hashlib.sha256(out.read_bytes()).hexdigest(), "frames": int(masks.shape[0]),
               "shape": list(masks.shape), "invalid_fraction_all": float(masks.mean()),
               "invalid_fraction_below_band": float(masks[:, TOP_BAND_ROWS:, :].mean()), "scenes": report}
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
