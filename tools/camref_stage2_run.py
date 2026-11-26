"""Runner of CAM-REF stage 2 (docs/plans/prereg_camonly_ref_stage2_2026-11-26.md): Policy F at 205 bars with F4 (F at P_T)
plus ONE of the five P_T -> P_S factors that F8 (F at P_S) carries together.

  F4D  F4 + delay U{1,2} steps            F4R  F4 + vehicle (T/W 3.5 raise, +-15 %, +-5 %, tau 0.02-0.04 s) and gain
  F4Y  F4 + yaw clip 1.047 rad/s (scale 3.0 kept)        (+-20 %) randomisation
  F4V  F4 + governor FREE 2.5 m/s          F4A  F4 + altitude 1.5 m / camera mount 0.08 m

Stage 1's runner (tools/camref_eval_run.py) builds, launches and checks every cell; this module only replaces its unit
constants (scope, preregistration, approval folder, plans, references, wall cap) and calls its main(). The F4 / F8 cells
of stage 1 (~/motar_tmp/camref_20261126/gpu1/eval, SHA-256 pinned) are the reference arms of the analysis; F4ID and F8ID
rerun their seed-4501 cells on this commit and must reproduce them episode for episode (else IDENTITY_FAIL, stop).

Stages (same chain and guards as stage 1): smoke (the five new arms at quota 1, 205 / 4501, never counted), identity (F4ID,
F8ID), eval (the five arms x seeds 4501-4503, block-major by seed: 15 cells). Wall cap 2.0 h shared by the three stages
(sub-caps 0.5 / 0.5 / 2.0 h).
usage: tools/camref_stage2_run.py scope | {check,smoke,identity,eval} [--approved-commit SHA] --out DIR
"""
import contextlib
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import camref_eval_run as R                                 # noqa: E402

PREREG = "docs/plans/prereg_camonly_ref_stage2_2026-11-26.md"
UNIT = "CAM-REF-2"
SCOPE = ("CAM-REF stage 2 v1: GPU evaluation of Policy F (ep25000 riskcap, f7022139...) at 205 bars, quota 8 per env at "
         "128 envs (1,024 episodes per cell), F4 (F at P_T) plus one P_T -> P_S factor each: F4D delay U{1,2}, F4R "
         "vehicle + gain randomisation, F4Y yaw clip 1.047 rad/s, F4V governor FREE 2.5 m/s, F4A altitude 1.5 m / mount "
         "0.08 m; smoke F4D, F4R, F4Y, F4V, F4A at quota 1 (205 / 4501); identity F4ID and F8ID against the CAM-REF stage-1 "
         "F4 and F8 cells 205 / 4501 (first 8 episodes per env), any difference IDENTITY_FAIL; eval F4D, F4R, F4Y, F4V, "
         "F4A x seeds 4501-4503 (15 cells, block-major by seed); stage-1 F4 / F8 cells 4501-4503 are the reference arms; "
         "wall cap 2.0 h shared by the three stages, cells that cannot finish inside it skipped; idle host; "
         "tools/camref_stage2_run.py over tools/camref_eval_run.py with eval_navrl_camref_arms.sh and tools/camref_entry.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camref2_20261126"
STAGE1_EVAL = pathlib.Path.home() / "motar_tmp" / "camref_20261126" / "gpu1" / "eval"
NEW_ARMS = ("F4D", "F4R", "F4Y", "F4V", "F4A")
REF_DIRS = {"CAMREF1_F4": STAGE1_EVAL / "F4" / "CAMREF_F4__205bars__seed%d",
            "CAMREF1_F8": STAGE1_EVAL / "F8" / "CAMREF_F8__205bars__seed%d"}
# every stage-1 F4 / F8 file the identity stage and the analysis read (eval_run_report COMPLETE, commit 358c363)
REF_SHA = {
    ("CAMREF1_F4", 4501): {"episodes.jsonl": "37953a360914144c863a949009fe520b97918e8d3edc2bd3d634a4f381fd0e8f",
                           "camref_episodes.jsonl": "681512963d13ca7c8de9aeaf4d8073b28e8b66c17ee28b63e18053cd6d65a6ed",
                           "receipt.json": "e834df2216e4e7c4ac8d51c8973bc62d2084e60d89e4bad7964c2a31dd45bd3e"},
    ("CAMREF1_F4", 4502): {"episodes.jsonl": "618096cab5014c27ee771c1d87d4e0a6af9bc60c85319313d45c863d0adc8d00",
                           "camref_episodes.jsonl": "05f9408e67aa3498fb0229d0994610a499fa94a2a72f7326800a17cd10e41750",
                           "receipt.json": "b744ed75d71cef2f2edd5b32717ed30551a941ceda55e2c60ef93e4c5a05ede0"},
    ("CAMREF1_F4", 4503): {"episodes.jsonl": "c4f7c5366a9e97061e08442551b998fe0e97d8cdeda28e33294a4f3be98c1444",
                           "camref_episodes.jsonl": "ccc992e6aba6bb061ae2dc7ab41e3300fd2468f33283610868bc32553357f3e4",
                           "receipt.json": "fa2bf549aa77e34e970310db5d90e34d019d2f9e9be94f423f768f9765a7a56a"},
    ("CAMREF1_F8", 4501): {"episodes.jsonl": "117e3c21c0bd3bfed1ef996006ad82a98d5682247b8391e3ff95ad7d34f56095",
                           "camref_episodes.jsonl": "d49de568f41bd4f0f37a3b303f17ade78cc3c7bff5695c5474f3756ee08ff34f",
                           "receipt.json": "417e526c80cebed7acd542efdb4b0d6a0322598cb2aa8863436af5e2466a79d5"},
    ("CAMREF1_F8", 4502): {"episodes.jsonl": "b1f67f787b71a7073576ed7d87af04acae36e64a228524cabe8f6523caabb15a",
                           "camref_episodes.jsonl": "87c7c97bb43366c551e88b4149b575fc0f03889166b732b2d63d03b7a0d4eef7",
                           "receipt.json": "a08f2b68f499eb3612af2adbf94ee33ba842822c1607140cca7c41f928d406e9"},
    ("CAMREF1_F8", 4503): {"episodes.jsonl": "6f674750c356b954b2696566cd61cae7db90319b7255597113f8f6c69d6c0d29",
                           "camref_episodes.jsonl": "74cf6ea8fda4e94c53b6065d6391efeb1f534efa7c7f1639be266dc69245a7d6",
                           "receipt.json": "b1934c32578201627e3e24e66355af77810ed3c8808382726e4407a8e674b69e"},
}
SMOKE_PLAN = tuple((a + "_smoke", R.DENSITY, 4501) for a in NEW_ARMS)
IDENTITY_PLAN = (("F4ID", R.DENSITY, 4501), ("F8ID", R.DENSITY, 4501))
IDENTITY_REF = {"F4ID": ("CAMREF1_F4", 4501), "F8ID": ("CAMREF1_F8", 4501)}
EVAL_ARMS = NEW_ARMS
MAX_HOURS_TOTAL = 2.0
STAGE_SUB_CAP_H = {"smoke": 0.5, "identity": 0.5, "eval": 2.0}
SMOKE_CONTROLS = ("every smoke cell OK with >= 1 finished episode",
                  "every smoke cell: [camref] rules telemetry with >= 1 capture or target contact; every rendered step-0 "
                  "bearing within +-28.35 deg (visible start)",
                  "F4D_smoke: delay draws of both 1 and 2 steps; the other four: no delay draw other than 0",
                  "F4R_smoke: a [veh-rand] init line; the other four: none",
                  "F4Y_smoke: a [yawclip] line with commands > 0; the other four: no [yawclip] line")
NEW_FILES = ("tools/camref_stage2_run.py", "tools/camref_stage2_analyse.py", "tests/test_camref_stage2.py")


def install():
    """Replace stage 1's unit constants in camref_eval_run (its functions read them at call time)."""
    R.PREREG, R.UNIT, R.SCOPE, R.UNIT_DIR = PREREG, UNIT, SCOPE, UNIT_DIR
    R.REF_DIRS = {**R.REF_DIRS, **REF_DIRS}
    R.REF_SHA = {**R.REF_SHA, **REF_SHA}
    R.SMOKE_PLAN, R.IDENTITY_PLAN, R.IDENTITY_REF, R.EVAL_ARMS = SMOKE_PLAN, IDENTITY_PLAN, IDENTITY_REF, EVAL_ARMS
    R.MAX_HOURS_TOTAL, R.STAGE_SUB_CAP_H, R.SMOKE_CONTROLS = MAX_HOURS_TOTAL, STAGE_SUB_CAP_H, SMOKE_CONTROLS
    R.COMMITTED_FILES = tuple(dict.fromkeys((PREREG,) + R.COMMITTED_FILES + NEW_FILES))
    R.NEW_FILES = tuple(dict.fromkeys(R.NEW_FILES + NEW_FILES))


PATCHED = ("PREREG", "UNIT", "SCOPE", "UNIT_DIR", "REF_DIRS", "REF_SHA", "SMOKE_PLAN", "IDENTITY_PLAN", "IDENTITY_REF",
           "EVAL_ARMS", "MAX_HOURS_TOTAL", "STAGE_SUB_CAP_H", "SMOKE_CONTROLS", "COMMITTED_FILES", "NEW_FILES")


@contextlib.contextmanager
def installed():
    """install() for the duration of a block, then stage 1's constants back (tests share one process)."""
    saved = {k: getattr(R, k) for k in PATCHED}
    try:
        install()
        yield R
    finally:
        for k, v in saved.items():
            setattr(R, k, v)


def main(argv=None):
    install()
    return R.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
