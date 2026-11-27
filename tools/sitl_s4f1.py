"""S4F-1 v1: a 2x2 factorial on S4's `ground` arm (PX4 SITL + Gazebo's own sensors). Preregistration: PREREG below.

Every episode is tools/sitl_s4.run_episode's with arm "ground", called unchanged: navrl_quad_s4 (visuals removed) with
Gazebo's gpu_ray LiDAR, RGB and depth cameras on a body-fixed mount, Frozen Policy F at 87 deg, and the 0.10 m world-z
ground filter applied in every cell. Only the world text differs, through the owned session's write_world:
  - factor A, target appearance: `lit` is S4's material exactly; `emissive` sets ambient = diffuse = specular = 0 and
    <emissive> to the target colour (0.88, 0.08, 0.045), so Gazebo's lighting cannot shade it;
  - factor G, ground: `rendered` is S4's world exactly (ground_plane + asphalt_plane); `unrendered` replaces the two
    includes by the same two models with their collisions only (same names, same geometry, same pose: the asphalt
    box top stays at z = 0.05 m, so the resting height is unchanged). Cameras and gpu_ray render visuals only.
The cell lit_rendered is S4's `ground` arm, byte for byte in its world text.

Scenes: the first 40 eligible 70-bar G8 r2 scenes (S4 `ground`'s schedule). Order: scene by scene, the 4 cells of a
scene in the order of a 4x4 Williams Latin square row (scene i uses row i mod 4), fixed below.
Development smokes: one episode per cell (scene 1 of the schedule, smoke01_<cell>), each COMPLETE and VALID at the same
HEAD and source bytes, before attempt01 (S3c/S4's smoke rule, applied per cell).
Disk: each episode's PX4 ULog is gzipped after the episode (after S4's dataman removal); the original is removed only
after the gzip stream has been decompressed and matched its SHA-256 and size; both are recorded.
Design review r1 (AMEND_FIRST) repairs: S4's plugin bytes pinned instead of its mtime rule, nothing rebuilt (MAJOR-1);
the replication of S4 attempt01's `ground` arm checked (stack, per-scene worlds, the world's model files; MAJOR-3);
the component smoke (C1-C9 PASS, its script pinned, this runner's bytes) gates every launch (MINOR-6); a per-episode
GPU-job and load check (MINOR-5); the start EKF-truth altitude gate on each cell smoke (MINOR-8); the reading labels
VOID / INSUFFICIENT / REPORTED and two Holm-corrected primary contrasts (MINOR-2, MINOR-3); manipulation checks
(MINOR-7). Reported, not gated.

  tools/sitl_s4f1.py --check | --smoke CELL | --launch | --analyze [ATTEMPT_DIR]
"""
import argparse
import collections
import gzip
import hashlib
import json
import math
import os
import pathlib
import re
import shutil
import sys
import time
import types

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))
import sitl_s3b_formal as SF                                    # noqa: E402
import sitl_s3c as S3C                                          # noqa: E402
import sitl_s4 as S4                                            # noqa: E402
import sitl_s4_gazebo as G                                      # noqa: E402

PROTOCOL = "s4f1-v1"
PREREG = "docs/plans/prereg_sitl_s4f1_v1_2026-11-08.md"
S4_AMENDMENT = "docs/plans/prereg_sitl_s3c_s4_amendment1_2026-11-07.md"
SCOPE = ("S4F-1 v1: Frozen Policy F through PX4 v1.14.3 SITL on navrl_quad_s4 (visuals removed) with Gazebo gpu_ray "
         "LiDAR, RGB and depth cameras (OpenGL rendering on display :1, no CUDA) as S4 arm ground (body-fixed mount, "
         "87 deg, 0.10 m world-z ground filter on); 2x2 factorial target appearance {lit, emissive} x ground "
         "{rendered, unrendered}; G8 r2 seed 4501, first 40 eligible 70-bar scenes, the 4 cells of each scene in a "
         "fixed Williams Latin-square order; after the component smoke passes C1-C9, development smokes "
         "smoke01_<cell> (the first scene once per cell, not read) then attempt01; S4's pinned plugin, not rebuilt; "
         "episode lateness bound 0.15 s including frame age; ULog gzip after each episode; no retries")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "s4f1_20261108"
ATTEMPT_DIR = UNIT_DIR / "attempt01"
S4_ARM = "ground"                        # S4's arm with the 0.10 m world-z ground filter: ON in every S4F-1 cell
CELLS = (("lit_rendered", "lit", "rendered"), ("emissive_rendered", "emissive", "rendered"),
         ("lit_unrendered", "lit", "unrendered"), ("emissive_unrendered", "emissive", "unrendered"))
CELL_NAMES = tuple(c for c, _, _ in CELLS)
FACTORS = {c: (a, g) for c, a, g in CELLS}
SMOKE_DIRS = {c: UNIT_DIR / ("smoke01_" + c) for c in CELL_NAMES}
# Williams (1949) balanced Latin square for 4 treatments (indices into CELLS): every cell once in each position per
# 4 scenes, and every ordered pair of different cells adjacent exactly once within a scene, over the 4 rows.
LATIN_SQUARE = ((0, 1, 3, 2), (1, 2, 0, 3), (2, 3, 1, 0), (3, 0, 2, 1))
N_SCENES = 40
BARS = 70
INVALID_ABORT_TOTAL = 16                 # more than 16 INVALID episodes of the 160 (10 %) stop the unit
DISK_FREE_LAUNCH_B = int(1.5e9)          # refuse to start below 1.5 GB free (expected use about 0.7 GB with gzip)
DISK_FREE_STOP_B = int(0.5e9)            # stop the unit (ABORTED) before an episode when below 0.5 GB free
ULOG_GLOB = "rootfs/log/*/*.ulg"
GZIP_LEVEL = 1                           # survey: gzip -1 ratio 0.33 on S4 attempt01 ULogs
S4_LEDGER = S4.ATTEMPT_DIR / "episodes.jsonl"
S4_LEDGER_SHA256 = "522f716e674d3f145ed3eed0003f6a8b1c2d6dfdb8f0f05399bba67bb240b397"   # S4 attempt01 (2026-11-07)
# S3b attempt01 (2026-11-06), pinned rather than only recorded (review r1 LOW-7)
S3B_LEDGER_SHA256 = "97b94dfd6d0c67ff01077787697aa87eee64e540b0195edaa3a78731a482e4fa"
# S4 attempt01's plugin (review r1 MAJOR-1): its bound .so and the source it was built from. The worktree checkout
# gives the committed .cc a new mtime, so S4's "older than its source" rule would refuse S4's own binary; with both
# pins matching, only that rule is dropped. Nothing is ever rebuilt into S4's PLUGIN_DIR.
S4_PLUGIN_SHA256 = "9b3109af0b737ad7a4975df21ba5231fdc35b5c4e4319dc0c95d9ff4a7806c92"
S4_PLUGIN_SOURCE_SHA256 = "661a4d71623aba293cb21feee569837bd5866c7f2357125f432b759196f8e5fa"
S4_PLUGIN_AGE = "is older than its source"         # the text of S4's plugin-age refusal (tools/sitl_s4.py)
# the world's own model:// resources (review r1 MAJOR-3), resolved on the PX4 tree's model path as the owned session
# sets it; files unchanged since 2026-11-04, before S4 ran. The runtime binding covers only the vehicle's resources.
WORLD_MODELS_URIS = "model://sun model://ground_plane model://asphalt_plane"
WORLD_MODEL_SHA256 = {
    "asphalt_plane/materials/scripts/asphalt.material":
        "ea014ce328ecfe17161cc31cdaca23e38405a887853905262a95264583dba783",
    "asphalt_plane/materials/textures/tarmac.png": "ef69f9572234d43b180fb5160bb17074aa5c4e8dbff54278c6bf2baa49c79aa7",
    "asphalt_plane/model-1_4.sdf": "4cf871d6e7930f67e274f231b5e880f6f5b685d7eae2661541f40e668af3cec3",
    "asphalt_plane/model.config": "ecc6ad081fd25d29c7618832befad06b4797dcdb607bc9895c6d65e2b49532e2",
    "asphalt_plane/model.sdf": "8492cd8f430277d45600b3f6387f46337b7c8d419989c49757b6be18d220accb",
    "ground_plane/model.config": "10f52e3c1bf4524e41746682e6ce33a7f24ed78efba11c05edcc342f5756110e",
    "ground_plane/model.sdf": "23d45725294ad6cfb18e2d191c041b3a5a0da09de1bfb3c6399f2b42771dbacd",
    "sun/model.config": "194e0880f1b9eea8597ab1d9d8e4dd42ef9dd5f4c86c24f2bfc701176946c284",
    "sun/model.sdf": "d73ffc90c30bad56242bbe2b934c12c39d900c5e4ac098196e32c970abfc6cee"}
# the component smoke (review r1 MINOR-6): its script is pinned here; its result must be PASS on C1-C9 and must have
# been produced with this runner's bytes, else every launch (cell smoke or attempt) refuses
COMPONENT_SMOKE = UNIT_DIR / "component_smoke.py"
COMPONENT_SMOKE_SHA256 = "14925afb963c27e6487f4133bd8f5e7f911b91578ac8c5ce51b185fa96f15e40"
COMPONENT_SMOKE_RESULT = UNIT_DIR / "component_smoke" / "result.json"
COMPONENT_CRITERIA = tuple("C%d" % i for i in range(1, 10))
START_ALT_TOL_M = 0.05     # cell smoke gate: |EKF - truth altitude| at the start check (review r1 MINOR-8)
MIN_COMPLETE_SCENES = 30   # fewer scenes VALID in all four cells: reading INSUFFICIENT (review r1 MINOR-3)
PRIMARY = (("capture", "main_A_emissive_minus_lit"), ("capture", "main_G_unrendered_minus_rendered"))
PRIMARY_ALPHA = 0.05       # Holm over the two primary contrasts (review r1 MINOR-2); everything else is descriptive
GPU_WAIT_S = 30.0          # a listed GPU job before an episode: one wait, then GPU_BUSY without running (MINOR-5)
NOT_RUN_OUTCOMES = ("NOT_RUN", "GPU_BUSY", "GPU_UNCHECKED")
OWN_SOURCES = ("tools/sitl_s4f1.py", "tests/test_sitl_s4f1.py", PREREG, S4_AMENDMENT) + S4.OWN_SOURCES

# ------------------------------------------------------------------------------------------------ world per cell
LIT_MATERIAL = ("<material><ambient>%.3f %.3f %.3f 1</ambient><diffuse>%.3f %.3f %.3f 1</diffuse>"
                "<specular>0 0 0 1</specular></material>" % (G.TARGET_RGB + G.TARGET_RGB))
EMISSIVE_MATERIAL = ("<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 1</specular>"
                     "<emissive>%.3f %.3f %.3f 1</emissive></material>" % G.TARGET_RGB)
# the collisions of the PX4 tree's ground_plane and asphalt_plane models (sitl_gazebo-classic/models/*/model.sdf),
# their visuals left out; same model and link names, no pose (both at the world origin as included)
GROUND_PLANE_COLLISION_ONLY = (
    "<model name='ground_plane'><static>true</static><link name='link'><collision name='collision'><geometry><plane>"
    "<normal>0 0 1</normal><size>100 100</size></plane></geometry><surface><contact>"
    "<collide_bitmask>65535</collide_bitmask></contact><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>"
    "</surface></collision></link></model>")
ASPHALT_PLANE_COLLISION_ONLY = (
    "<model name='asphalt_plane'><static>true</static><link name='link'><collision name='collision'><geometry><box>"
    "<size>200 200 .1</size></box></geometry></collision></link></model>")
GROUND_INCLUDES = (("ground_plane", re.compile(r"<include>\s*<uri>model://ground_plane</uri>\s*</include>"),
                    GROUND_PLANE_COLLISION_ONLY),
                   ("asphalt_plane", re.compile(r"<include>\s*<uri>model://asphalt_plane</uri>\s*</include>"),
                    ASPHALT_PLANE_COLLISION_ONLY))


def _sha_text(text):
    return hashlib.sha256(text.encode()).hexdigest()


def world_problems(s4_text):
    """What S4F-1 replaces must be in S4's world text exactly once (else the cell world would silently equal S4's)."""
    p = []
    if s4_text.count(LIT_MATERIAL) != 1:
        p.append("S4 target material found %d times, expected once" % s4_text.count(LIT_MATERIAL))
    if s4_text.count(EMISSIVE_MATERIAL) or "<emissive>" in s4_text:
        p.append("S4 world text already holds an emissive material")
    for name, pat, _ in GROUND_INCLUDES:
        n = len(pat.findall(s4_text))
        if n != 1:
            p.append("include of model://%s found %d times, expected once" % (name, n))
    return p


def world_text_cell(cell, s4_text):
    """The cell's world from S4's world text (tools/sitl_s4_gazebo.world_text_s4 of the S3b world). lit_rendered
    returns S4's text unchanged; the others replace only the target material and/or the two ground includes."""
    if cell not in FACTORS:
        raise ValueError("unknown cell %r" % (cell,))
    p = world_problems(s4_text)
    if p:
        raise ValueError("S4 world text is not as expected: %s" % "; ".join(p))
    appearance, ground = FACTORS[cell]
    out = s4_text
    if appearance == "emissive":
        out = out.replace(LIT_MATERIAL, EMISSIVE_MATERIAL, 1)
    if ground == "unrendered":
        for _, pat, repl in GROUND_INCLUDES:
            out = pat.sub(repl, out, count=1)
    return out


def cell_os(OS, cell, world_record):
    """The owned-session module's namespace with OwnedSession.write_world writing the cell's world; the S4 world it
    replaced and the written one are recorded (SHA-256) in world_record."""
    class CellSession(OS.OwnedSession):
        def write_world(self, name, text):
            new = world_text_cell(cell, text)
            world_record.update(cell=cell, s4_world_sha256=_sha_text(text), world_sha256=_sha_text(new),
                                equals_s4_world=(new == text))
            return super().write_world(name, new)
    ns = types.SimpleNamespace(**{k: getattr(OS, k) for k in dir(OS) if not k.startswith("__")})
    ns.OwnedSession = CellSession
    return ns


# ------------------------------------------------------------------------------------------------ schedule
def scene_envs(g8=SF.G8):
    return SF.eligible_envs(SF.read_jsonl(g8 / "zero" / (SF.CELL % BARS) / "kroc2027_episodes.jsonl"))[:N_SCENES]


def build_schedule(envs):
    out = []
    for i, e in enumerate(envs):
        row = LATIN_SQUARE[i % len(LATIN_SQUARE)]
        for pos, ci in enumerate(row):
            cell, a, g = CELLS[ci]
            out.append({"arm": cell, "bars": BARS, "env": e, "scene_index": i, "position": pos,
                        "square_row": i % len(LATIN_SQUARE), "appearance": a, "ground": g})
    return out


def schedule(g8=SF.G8):
    return build_schedule(scene_envs(g8))


def latin_square_problems(square=LATIN_SQUARE, n=len(CELLS)):
    """Rows are permutations, each cell once per position, and each ordered pair of different cells adjacent exactly
    once (first-order carry-over balance)."""
    p = []
    if len(square) != n or any(sorted(r) != list(range(n)) for r in square):
        return ["not an %d x %d square of permutations" % (n, n)]
    for pos in range(n):
        if sorted(r[pos] for r in square) != list(range(n)):
            p.append("position %d does not hold every cell once" % pos)
    pairs = collections.Counter((r[j], r[j + 1]) for r in square for j in range(n - 1))
    if len(pairs) != n * (n - 1) or set(pairs.values()) != {1}:
        p.append("carry-over not balanced: %s" % dict(pairs))
    return p


def design_problems(sched, envs=None):
    """The recorded schedule is the fixed design: N_SCENES scenes in ascending env order, each scene's 4 items the
    Latin-square row of its index, and (if given) these envs."""
    p = []
    if len(sched) != len(CELLS) * N_SCENES:
        p.append("schedule has %d items, expected %d" % (len(sched), len(CELLS) * N_SCENES))
    got_envs = [s.get("env") for s in sched[::len(CELLS)]]
    if got_envs != sorted(set(got_envs)):
        p.append("scenes are not in ascending, distinct env order")
    if envs is not None and got_envs != list(envs)[:N_SCENES]:
        p.append("scenes differ from the first %d eligible %d-bar scenes" % (N_SCENES, BARS))
    want = build_schedule(got_envs)
    if [(s.get("arm"), s.get("env")) for s in sched] != [(s["arm"], s["env"]) for s in want]:
        p.append("cell order is not the fixed Latin-square order")
    return p


def smoke_items(sched, cell):
    """The development smoke of a cell: its item in the first scene (every cell occurs in the first scene)."""
    first = sched[0]["env"]
    items = [s for s in sched if s["arm"] == cell and s["env"] == first]
    if len(items) != 1:
        raise ValueError("no single first-scene item for cell %r" % (cell,))
    return items


# ------------------------------------------------------------------------------------------------ ULog gzip
def _hash_stream(f):
    h, n = hashlib.sha256(), 0
    for chunk in iter(lambda: f.read(1 << 20), b""):
        h.update(chunk)
        n += len(chunk)
    return h.hexdigest(), n


def _gzip_copy(src, dst_raw, level):
    with open(src, "rb") as fi, gzip.GzipFile(filename="", mode="wb", compresslevel=level, fileobj=dst_raw,
                                              mtime=0) as gz:
        shutil.copyfileobj(fi, gz, 1 << 20)


def gzip_verified(src, root, level=GZIP_LEVEL, compress=_gzip_copy):
    """src -> src.gz. The original is removed only after the finished .gz has been decompressed and matched the
    original's SHA-256 and size (and the original was unchanged meanwhile). An existing .gz is never overwritten.
    On any failure the original stays and the partial file is removed. Returns the record."""
    src, root = pathlib.Path(src), pathlib.Path(root)
    gz, part = src.with_name(src.name + ".gz"), src.with_name(src.name + ".gz.part")
    rec = {"path": str(src.relative_to(root)), "gz_path": str(gz.relative_to(root)), "verified": False,
           "original_removed": False}
    try:
        if gz.exists() or part.exists():
            raise FileExistsError("refusing to overwrite %s" % (gz if gz.exists() else part))
        with open(src, "rb") as f:
            rec["sha256"], rec["bytes"] = _hash_stream(f)
        with open(part, "xb") as raw:
            compress(src, raw, level)
            raw.flush()
            os.fsync(raw.fileno())
        with gzip.open(part, "rb") as f:
            got = _hash_stream(f)
        if got != (rec["sha256"], rec["bytes"]):
            raise ValueError("gzip round trip differs from the original (%s, %d bytes)" % got)
        if SF.sha256_file(src) != rec["sha256"]:
            raise ValueError("original changed during compression")
        if gz.exists():
            raise FileExistsError("refusing to overwrite %s" % gz)
        os.rename(part, gz)
        dfd = os.open(str(gz.parent), os.O_RDONLY)
        try:
            os.fsync(dfd)                                 # the rename is durable before the original goes (LOW-5)
        finally:
            os.close(dfd)
        rec.update(gz_sha256=SF.sha256_file(gz), gz_bytes=gz.stat().st_size, verified=True)
        src.unlink()
        rec["original_removed"] = True
    except Exception as e:                                               # noqa: BLE001 - recorded, original kept
        rec["error"] = "%s: %s" % (e.__class__.__name__, e)
        if part.exists() and not rec["verified"]:
            part.unlink()
    return rec


def gzip_ulogs(ep_dir, level=GZIP_LEVEL):
    ep_dir = pathlib.Path(ep_dir)
    files = [gzip_verified(f, ep_dir, level) for f in sorted(ep_dir.glob(ULOG_GLOB))] if ep_dir.is_dir() else []
    return {"count": len(files), "all_verified": bool(files) and all(r["verified"] for r in files), "files": files,
            "level": level}


def verify_ulog_record(ep_dir, rec):
    """A recorded ULog gzip still decompresses to the recorded original (SHA-256 and size), with the recorded .gz
    SHA-256. Returns a list of problems."""
    gz = pathlib.Path(ep_dir) / rec.get("gz_path", "")
    if not rec.get("verified"):
        return ["%s: not verified at the episode (%s)" % (rec.get("path"), rec.get("error"))]
    if not gz.is_file():
        return ["%s missing" % gz]
    p = []
    if SF.sha256_file(gz) != rec.get("gz_sha256"):
        p.append("%s: gzip SHA-256 differs from the record" % gz)
    try:
        with gzip.open(gz, "rb") as f:
            got = _hash_stream(f)
    except (OSError, EOFError) as e:
        return p + ["%s: does not decompress (%s)" % (gz, e)]
    if got != (rec.get("sha256"), rec.get("bytes")):
        p.append("%s: decompresses to other bytes than the recorded original" % gz)
    return p


# ------------------------------------------------------------------------------------------------ live episode
def run_episode(B, OS, V0, spec, ep_dir, policy, cell, link_factory=None, client_factory=None):
    """tools/sitl_s4.run_episode with arm "ground" in the cell's world, then the ULog gzip."""
    if cell not in FACTORS:
        raise ValueError("unknown cell %r" % (cell,))
    world = {}
    res = S4.run_episode(B, cell_os(OS, cell, world), V0, spec, ep_dir, policy, S4_ARM, link_factory=link_factory,
                         client_factory=client_factory)
    res["s4_arm"] = res.pop("arm", None)
    a, g = FACTORS[cell]
    res.update(arm=cell, cell=cell, appearance=a, ground=g, cell_world=world or None)
    res["ulog"] = gzip_ulogs(ep_dir)
    return res


def disk_free(path=UNIT_DIR):
    p = pathlib.Path(path)
    while not p.exists():
        p = p.parent
    return shutil.disk_usage(p).free


def disk_problems(free=None, need=DISK_FREE_LAUNCH_B):
    free = disk_free() if free is None else free
    return [] if free >= need else ["disk free %.2f GB is below the %.2f GB launch floor" % (free / 1e9, need / 1e9)]


def smokes_problems(head, sources_sha, smoke_dirs=None):
    """attempt01 needs every cell's smoke COMPLETE and VALID at this HEAD and these source bytes (S3C.smoke_problems
    per cell), and each smoke must be the episode of its own cell."""
    smoke_dirs = SMOKE_DIRS if smoke_dirs is None else smoke_dirs
    p = []
    for cell in CELL_NAMES:
        d = pathlib.Path(smoke_dirs[cell])
        q = S3C.smoke_problems(d, head, sources_sha)
        if not q:
            att = json.loads((d / "attempt.json").read_text())
            rows = SF.read_jsonl(d / "episodes.jsonl")
            if att.get("smoke_cell") != cell or att.get("protocol") != PROTOCOL or rows[0].get("arm") != cell:
                q = ["the smoke in %s is not cell %s's" % (d, cell)]
            else:
                dz = start_alt_diff(rows[0])
                if dz is None or abs(dz) > START_ALT_TOL_M:
                    q = ["start EKF - truth altitude %r m is not within %.2f m (rangefinder height reference)"
                         % (dz, START_ALT_TOL_M)]
        p += ["smoke %s: %s" % (cell, x) for x in q]
    return p


def start_alt_diff(row):
    """EKF minus truth altitude at the start check. The airframe's height reference is the PX4 rangefinder
    (EKF2_HGT_REF 2), a physics ray sensor that still sees the collision-only floor; S4 attempt01 read -0.013 to
    +0.020 m, a barometer fallback about 0.1-0.15 m (review r1 MINOR-8)."""
    sc = row.get("start_check") or {}
    try:
        return float(sc["ekf_alt_m"]) - float(sc["truth_alt_m"])
    except (KeyError, TypeError, ValueError):
        return None


def plugin_pin_problems(s4_problems, so=None, src=None, pins=None):
    """S4's launch refusals with its plugin-age rule replaced by S4 attempt01's pins (review r1 MAJOR-1): if the .so and
    the source are S4's bytes, only the "older than its source" refusal is dropped; otherwise a pin refusal is added
    and the age refusal stays. Nothing is rebuilt."""
    so = S4.PLUGIN_DIR / "libmotar_s4_bridge.so" if so is None else pathlib.Path(so)
    src = S4.PLUGIN_SRC if src is None else pathlib.Path(src)
    want_so, want_src = pins or (S4_PLUGIN_SHA256, S4_PLUGIN_SOURCE_SHA256)
    got_so = SF.sha256_file(so) if so.is_file() else None
    got_src = SF.sha256_file(src) if src.is_file() else None
    p = list(s4_problems)
    if got_so == want_so and got_src == want_src:
        return [x for x in p if S4_PLUGIN_AGE not in x]
    return p + ["S4 plugin %s (%s) or its source %s (%s) is not S4 attempt01's pinned bytes; nothing is rebuilt"
                % (so, (got_so or "missing")[:12], src, (got_src or "missing")[:12])]


def world_model_files(models=None):
    """{key: {path, sha256}} for every file of the world's model:// resources, resolved as the owned session does."""
    import sitl_owned_session as OS
    models = models or (OS.PX4_ROOT / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models")
    found = OS._model_resources(WORLD_MODELS_URIS, str(models))
    return {k: {"path": str(v), "sha256": SF.sha256_file(v)} for k, v in sorted(found.items())}


def world_model_problems(files, pins=None):
    pins = WORLD_MODEL_SHA256 if pins is None else pins
    got = {k: (v.get("sha256") if isinstance(v, dict) else None) for k, v in (files or {}).items()}
    bad = sorted(k for k in set(got) | set(pins) if got.get(k) != pins.get(k))
    return ["world model files differ from the pins: %s" % bad] if bad else []


def component_smoke_problems(result=None, runner=None, script_sha=None):
    """Every launch needs the component smoke's result: verdict PASS with C1-C9 true, produced by the pinned script
    and by this runner's bytes (review r1 MINOR-6); otherwise the smoke is rerun or the unit amended."""
    result = COMPONENT_SMOKE_RESULT if result is None else pathlib.Path(result)
    runner = TOOLS / "sitl_s4f1.py" if runner is None else pathlib.Path(runner)
    script_sha = COMPONENT_SMOKE_SHA256 if script_sha is None else script_sha
    if not result.is_file():
        return ["no component smoke result %s" % result]
    try:
        r = json.loads(result.read_text())
    except ValueError as e:
        return ["component smoke result unreadable: %s" % e]
    p = []
    crit = r.get("criteria") or {}
    if r.get("verdict") != "PASS" or any(crit.get(c) is not True for c in COMPONENT_CRITERIA):
        p.append("component smoke is not PASS on C1-C9: %s %s" % (r.get("verdict"), crit))
    if r.get("script_sha256") != script_sha:
        p.append("component smoke ran another script (%s), not the pinned one" % str(r.get("script_sha256"))[:12])
    if r.get("runner_sha256") != SF.sha256_file(runner):
        p.append("component smoke used another runner (%s): rerun it with this one" % str(r.get("runner_sha256"))[:12])
    return p


def gpu_jobs():
    """(ok, jobs, raw): compute processes listed by nvidia-smi other than S4's allowed remote desktop. ok is None
    when nvidia-smi fails (the check would be vacuous)."""
    import subprocess
    try:
        d = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,process_name", "--format=csv,noheader"],
                           capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, [], str(e)
    if d.returncode != 0:
        return None, [], "nvidia-smi rc %d" % d.returncode
    jobs = [ln.strip() for ln in d.stdout.splitlines() if ln.strip() and not ln.strip().endswith(S4.GPU_ALLOWED)]
    return not jobs, jobs, d.stdout.strip()


def host_snapshot(gpu_fn=None):
    ok, jobs, raw = (gpu_fn or gpu_jobs)()
    try:
        load = pathlib.Path("/proc/loadavg").read_text().split()[:3]
    except OSError:
        load = None
    return {"gpu_ok": ok, "gpu_jobs": jobs, "gpu_raw": raw[:300], "loadavg": load, "t": time.time()}


def make_run_one(B, OS, V0, policy, out_dir, free_fn=None, gpu_fn=None, wait_s=GPU_WAIT_S, episode_fn=None):
    """The per-episode call: the disk stop (DISK_LOW raises, so run_unit ends ABORTED with no ledger row), the GPU-job
    and load snapshot (a listed job: one wait, then GPU_BUSY without running; a failing nvidia-smi: GPU_UNCHECKED),
    then the episode."""
    free_fn = free_fn or disk_free
    episode_fn = episode_fn or run_episode

    def run_one(i, item, spec):
        free = free_fn(out_dir)
        if free < DISK_FREE_STOP_B:
            raise RuntimeError("DISK_LOW: %.2f GB free before episode %d" % (free / 1e9, i))
        snap = host_snapshot(gpu_fn)
        if snap["gpu_ok"] is not True and wait_s > 0:
            time.sleep(wait_s)
            snap = dict(host_snapshot(gpu_fn), waited_s=wait_s)
        if snap["gpu_ok"] is not True:
            return {"outcome": "GPU_BUSY" if snap["gpu_ok"] is False else "GPU_UNCHECKED", "host_gpu_before": snap}
        res = episode_fn(B, OS, V0, spec, pathlib.Path(out_dir) / ("ep%03d" % i), policy, item["arm"])
        res["host_gpu_before"] = snap
        return res
    return run_one


def launch_extra_problems(smoke_cell, head, sources_sha, s4_problems=None, free=None, smoke_dirs=None, plugin=None,
                          world_models=None, component=None):
    """S4's refusals (DISPLAY, port, GPU; the plugin by S4's pins), the disk floor, the world's model files by their
    pins, the component smoke, and for attempt01 the four cell smokes."""
    p = plugin_pin_problems(S4.s4_extra_problems() if s4_problems is None else s4_problems, *(plugin or (None, None)))
    p += disk_problems(free)
    try:
        wm = world_model_files() if world_models is None else world_models
        p += world_model_problems(wm)
    except Exception as e:                                               # noqa: BLE001 - a refusal
        p.append("world model files unresolved: %s" % e)
    p += component_smoke_problems(**(component or {}))
    if smoke_cell is None:
        p += smokes_problems(head, sources_sha, smoke_dirs)
    elif smoke_cell not in FACTORS:
        p.append("unknown smoke cell %r" % (smoke_cell,))
    return p


def check():
    inputs = SF.input_problems()
    sched = schedule() if not inputs else []
    out = {"input_problems": inputs, "schedule": collections.Counter(s["arm"] for s in sched),
           "design_problems": design_problems(sched) if sched else ["no schedule"],
           "latin_square_problems": latin_square_problems(),
           "s4_launch_problems": plugin_pin_problems(S4.s4_extra_problems()), "disk_problems": disk_problems(),
           "world_model_problems": world_model_problems(world_model_files()),
           "component_smoke_problems": component_smoke_problems(),
           "disk_free_gb": disk_free() / 1e9,
           "smokes": {c: S3C.smoke_problems(SMOKE_DIRS[c]) for c in CELL_NAMES},
           "display": os.environ.get("DISPLAY"), "environment_problems": SF.environment_report()[1]}
    print(json.dumps(out, indent=1, default=str))
    return out


def _safe(fn):
    try:
        return fn()
    except Exception as e:                                               # noqa: BLE001 - recorded; launch refuses
        return {"error": "%s: %s" % (e.__class__.__name__, e)}


def launch(smoke_cell=None):
    import sitl_dev08_attempt as AT
    sources = sorted(set(OWN_SOURCES) | set(AT.SOURCES))
    smoke = smoke_cell is not None
    if smoke and smoke_cell not in FACTORS:
        raise SystemExit("refused:\n  unknown smoke cell %r" % (smoke_cell,))
    out_dir = SMOKE_DIRS[smoke_cell] if smoke else ATTEMPT_DIR
    head = AT.git_state(ROOT)["head"]
    sources_sha = {s: SF.sha256_file(ROOT / s) if (ROOT / s).is_file() else None for s in sources}
    extra = launch_extra_problems(smoke_cell, head, sources_sha)
    so = S4.PLUGIN_DIR / "libmotar_s4_bridge.so"
    px4m = (pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/"
                         "sitl_gazebo-classic/models") / S4.MODEL / (S4.MODEL + ".sdf"))
    attempt_extra = {"protocol": PROTOCOL, "prereg": PREREG, "cells": CELLS, "latin_square": LATIN_SQUARE,
                     "smoke_cell": smoke_cell, "s4_arm": S4_ARM, "display": os.environ.get("DISPLAY"),
                     "plugin_sha256": SF.sha256_file(so) if so.is_file() else None,
                     "plugin_source_sha256": SF.sha256_file(S4.PLUGIN_SRC),
                     "model_sdf_sha256": SF.sha256_file(px4m) if px4m.is_file() else None,
                     "materials": {"lit": LIT_MATERIAL, "emissive": EMISSIVE_MATERIAL},
                     "ground_unrendered": [r for _, _, r in GROUND_INCLUDES], "disk_free_b_at_launch": disk_free(),
                     "plugin_pins": {"so": S4_PLUGIN_SHA256, "source": S4_PLUGIN_SOURCE_SHA256,
                                     "rule": "S4's mtime rule replaced by the pins; not rebuilt (review r1 MAJOR-1)"},
                     "world_models": _safe(world_model_files),
                     "component_smoke": {"result": str(COMPONENT_SMOKE_RESULT),
                                         "result_sha256": SF.sha256_file(COMPONENT_SMOKE_RESULT)
                                         if COMPONENT_SMOKE_RESULT.is_file() else None,
                                         "script_sha256": COMPONENT_SMOKE_SHA256},
                     "start_alt_tol_m": START_ALT_TOL_M, "host_at_launch": host_snapshot()}
    sched_fn = (lambda: smoke_items(schedule(), smoke_cell)) if smoke else schedule

    def run_one_fn(B, OS, V0, policy):
        return make_run_one(B, OS, V0, policy, out_dir)
    S3C.launch_unit(SCOPE, UNIT_DIR, out_dir, sources, sched_fn, attempt_extra, run_one_fn, extra_problems=extra,
                    invalid_abort=INVALID_ABORT_TOTAL, smoke_dir=None, smoke=smoke)


# ------------------------------------------------------------------------------------------------ analysis
OUTCOMES = ("capture", "crash", "timeout")


def sign_test_p(n_pos, n_neg):
    """Exact two-sided sign test (= exact McNemar on discordant pairs): 2 P(X <= min) for X ~ Bin(n, 1/2), capped
    at 1; no discordant pair gives 1."""
    n = int(n_pos) + int(n_neg)
    if n == 0:
        return 1.0
    k = min(int(n_pos), int(n_neg))
    return min(1.0, 2.0 * sum(math.comb(n, i) for i in range(k + 1)) / 2.0 ** n)


def scene_table(rows):
    """{env: {cell: outcome}} over VALID rows. Refuses an unknown cell, a repeated (cell, env) and a VALID row whose
    outcome is not capture, crash or timeout."""
    table = collections.defaultdict(dict)
    seen = set()
    for r in rows:
        cell, env = r.get("arm"), r.get("env")
        if cell not in FACTORS:
            raise ValueError("unknown cell %r" % (cell,))
        if (cell, env) in seen:
            raise ValueError("cell %s scene %r appears twice" % (cell, env))
        seen.add((cell, env))
        if r.get("valid"):
            if r.get("outcome") not in OUTCOMES:
                raise ValueError("VALID row with outcome %r" % (r.get("outcome"),))
            table[env][cell] = r["outcome"]
    return dict(table)


def _contrast(vals):
    pos, neg = sum(1 for v in vals if v > 0), sum(1 for v in vals if v < 0)
    return {"mean_pp": 100.0 * sum(vals) / len(vals) if vals else None, "scenes_positive": pos,
            "scenes_negative": neg, "scenes_zero": len(vals) - pos - neg,
            "sign_test_p_two_sided": sign_test_p(pos, neg)}


def _mcnemar(y, envs, x_cell, y_cell):
    b = sum(1 for e in envs if y[e][x_cell] and not y[e][y_cell])
    c = sum(1 for e in envs if y[e][y_cell] and not y[e][x_cell])
    return {"cells": [x_cell, y_cell], "pairs": len(envs), "x_only": b, "y_only": c,
            "diff_pp": 100.0 * (b - c) / len(envs) if envs else None, "mcnemar_exact_p_two_sided": sign_test_p(b, c)}


def paired_effects(rows, outcome="capture"):
    """Per scene VALID in all four cells, y = 1 if the episode's outcome is `outcome`. Main effect of A
    (emissive - lit, averaged over G), of G (unrendered - rendered, averaged over A) and the interaction
    (A effect when unrendered - A effect when rendered), each with an exact two-sided sign test over scenes; plus
    the four simple effects with exact McNemar tests."""
    if outcome not in OUTCOMES:
        raise ValueError("outcome %r" % (outcome,))
    table = scene_table(rows)
    envs = sorted(e for e, d in table.items() if set(d) == set(CELL_NAMES))
    y = {e: {c: int(table[e][c] == outcome) for c in CELL_NAMES} for e in envs}
    a_eff = [(d["emissive_rendered"] + d["emissive_unrendered"] - d["lit_rendered"] - d["lit_unrendered"]) / 2.0
             for d in (y[e] for e in envs)]
    g_eff = [(d["lit_unrendered"] + d["emissive_unrendered"] - d["lit_rendered"] - d["emissive_rendered"]) / 2.0
             for d in (y[e] for e in envs)]
    axg = [(d["emissive_unrendered"] - d["lit_unrendered"]) - (d["emissive_rendered"] - d["lit_rendered"])
           for d in (y[e] for e in envs)]
    return {"outcome": outcome, "scenes_valid_in_all_cells": len(envs), "scenes_with_any_valid": len(table),
            "envs": envs, "rate_pct": {c: (100.0 * sum(y[e][c] for e in envs) / len(envs)) if envs else None
                                       for c in CELL_NAMES},
            "main_A_emissive_minus_lit": _contrast(a_eff), "main_G_unrendered_minus_rendered": _contrast(g_eff),
            "interaction_AxG": _contrast(axg),
            "simple": {"A_given_rendered": _mcnemar(y, envs, "emissive_rendered", "lit_rendered"),
                       "A_given_unrendered": _mcnemar(y, envs, "emissive_unrendered", "lit_unrendered"),
                       "G_given_lit": _mcnemar(y, envs, "lit_unrendered", "lit_rendered"),
                       "G_given_emissive": _mcnemar(y, envs, "emissive_unrendered", "emissive_rendered")}}


def sensor_summary(ar):
    """tools/sitl_s4.analyze's per-arm camera recall and target pixel statistics, for one cell (same definitions,
    same upper median)."""
    valid = [r for r in ar if r["valid"] and r.get("sensor")]
    mv = sum(r["sensor"]["steps_motar_visible"] for r in valid)
    cv = sum(r["sensor"]["steps_camera_visible_when_motar_visible"] for r in valid)
    lv = sum(r["sensor"]["steps_lidar_only_visible_when_motar_visible"] for r in valid)

    def med(k):
        v = sorted(r["sensor"][k] for r in valid if r["sensor"].get(k) is not None)
        return float(v[len(v) // 2]) if v else None
    return {"detector_recall_steps": {
                "denominator": "steps where MOTAR's shadow renderer (yaw-level camera) has >= 2 target pixels",
                "motar_visible_steps": mv, "camera_visible_of_those": cv, "lidar_only_visible_of_those": lv,
                "camera_recall": (cv / mv) if mv else None},
            "sensor_medians_over_episodes": {k: med(k) for k in (
                "colour_px_median_when_motar_visible", "motar_px_median_when_motar_visible", "frame_age_median_s",
                "lidar_abs_diff_median_m", "depth_abs_diff_median_m")},
            "frame_age_max_s": max((r["sensor"]["frame_age_max_s"] for r in valid), default=None),
            "hover_guard_failures": sum(1 for r in ar if "hover guard" in str(r.get("error", "")))}


def cell_summary(ar, isaac, s3b):
    summ = SF.block_summary(ar, isaac)
    n = summ["valid"]
    summ["invalid_n"] = summ["scheduled"] - n                     # by reason in summ["invalid"] (review r1 MINOR-3)
    for k in OUTCOMES:
        c = summ["sitl"].get(k, 0)
        summ["%s_n" % k] = c
        summ["%s_pct" % k] = (100.0 * c / n) if n else None
        summ["%s_ci95_pct" % k] = SF.clopper_pearson(c, n)
    summ.update(sensor_summary(ar))
    summ["vs_s3b_same_scenes"] = S3C.matched(ar, s3b)
    dz = [start_alt_diff(r) for r in ar if r.get("valid")]
    dz = sorted(x for x in dz if x is not None)
    summ["start_ekf_minus_truth_alt_m"] = ({"n": len(dz), "min": dz[0], "median": dz[len(dz) // 2], "max": dz[-1]}
                                           if dz else None)
    return summ


def s4_reference(path=S4_LEDGER, want_sha=S4_LEDGER_SHA256):
    """S4 attempt01's VALID `ground` rows by env, bound by the pinned SHA-256 of its ledger."""
    path = pathlib.Path(path)
    if not path.is_file():
        return {}, {"status": "S4_NOT_AVAILABLE", "path": str(path)}
    sha = SF.sha256_file(path)
    if sha != want_sha:
        return {}, {"status": "S4_LEDGER_SHA_MISMATCH", "path": str(path), "sha256": sha, "want": want_sha}
    rows = {r["env"]: r for r in SF.read_jsonl(path) if r.get("arm") == S4_ARM and r.get("valid")}
    return rows, {"status": "BOUND", "path": str(path), "sha256": sha}


def s3b_reference(s3b_attempt=SF.ATTEMPT_DIR, want_sha=S3B_LEDGER_SHA256):
    """tools/sitl_s3c.s3b_reference with its ledger SHA-256 pinned (review r1 LOW-7)."""
    rows, ref = S3C.s3b_reference(s3b_attempt)
    if ref.get("status") == "BOUND" and ref.get("sha256") != want_sha:
        return {}, dict(ref, status="S3B_LEDGER_SHA_MISMATCH", want=want_sha)
    return rows, ref


def vs_s4_ground(ar, s4rows):
    """Descriptive: the lit_rendered cell against S4 attempt01's ground arm on the scenes VALID in both."""
    pairs = [(s4rows[r["env"]]["outcome"], r["outcome"]) for r in ar if r["valid"] and r["env"] in s4rows]
    cross = collections.Counter("%s->%s" % p for p in pairs)
    return {"pairs": len(pairs), "same_outcome": sum(1 for a, b in pairs if a == b),
            "s4_capture": sum(1 for a, _ in pairs if a == "capture"),
            "s4f1_capture": sum(1 for _, b in pairs if b == "capture"),
            "capture_s4_only": sum(1 for a, b in pairs if a == "capture" and b != "capture"),
            "capture_s4f1_only": sum(1 for a, b in pairs if b == "capture" and a != "capture"),
            "s4_to_s4f1_outcomes": dict(sorted(cross.items())), "label": "descriptive; no test"}


def _bound_world(r):
    return (((r.get("runtime_binding") or {}).get("files") or {}).get("world") or {}).get("sha256")


def _core(r):
    return json.dumps(SF.binding_core(r.get("runtime_binding")), sort_keys=True)


def replication(rows, s4rows, s4_ref, attempt=None, world_models_now=None):
    """Does lit_rendered run S4 attempt01's `ground` arm (review r1 MAJOR-3)? The stack binding (world excluded, S4
    plugin included) of every VALID S4F-1 episode equals S4 ground's single core; on each env VALID in both,
    lit_rendered's bound world SHA-256 equals S4's; the world's model files recorded at launch equal today's.
    Status DIFFERS on any mismatch; INCOMPLETE when the model files could not be compared or no world was compared
    (never SAME_STACK_AND_WORLDS then; review r2 N4)."""
    out = {"status": None, "problems": [], "worlds_compared": 0, "worlds_equal": 0, "models_compared": False}
    if s4_ref.get("status") != "BOUND":
        out["status"] = "S4_NOT_AVAILABLE"
        return out
    p = out["problems"]
    s4_cores = {_core(r) for r in s4rows.values()}
    ours = {_core(r) for r in rows if r.get("valid")}
    if len(s4_cores) != 1:
        p.append("S4 attempt01's ground arm has %d binding cores" % len(s4_cores))
    elif ours and ours != s4_cores:
        ref = json.loads(next(iter(s4_cores)))
        keys = sorted({k for c in ours for k, v in json.loads(c).items() if ref.get(k) != v}
                      | {k for c in ours for k in set(ref) - set(json.loads(c))})
        p.append("the stack binding differs from S4 attempt01's ground arm: %s" % keys)
    lr = [r for r in rows if r.get("arm") == "lit_rendered" and r.get("valid") and r.get("env") in s4rows]
    bad = [r["env"] for r in lr if _bound_world(r) is None or _bound_world(r) != _bound_world(s4rows[r["env"]])]
    out["worlds_compared"], out["worlds_equal"] = len(lr), len(lr) - len(bad)
    if bad:
        p.append("lit_rendered world SHA-256 differs from S4 attempt01's on envs %s" % bad)
    if attempt is not None and attempt.get("world_models") and world_models_now:
        then = {k: (v.get("sha256") if isinstance(v, dict) else None)
                for k, v in (attempt.get("world_models") or {}).items()}
        now = {k: (v.get("sha256") if isinstance(v, dict) else None) for k, v in world_models_now.items()}
        out["models_compared"] = True
        if then != now:
            p.append("the world's model files changed since the launch: %s"
                     % sorted(k for k in set(then) | set(now) if then.get(k) != now.get(k)))
    out["stack_compared"] = len(s4_cores) == 1 and bool(ours)
    if p:
        out["status"] = "DIFFERS"
    elif not (out["models_compared"] and out["worlds_compared"] and out["stack_compared"]):
        out["status"] = "INCOMPLETE"
    else:
        out["status"] = "SAME_STACK_AND_WORLDS"
    return out


def record_problems(rows):
    """Cell-world and binding checks over VALID rows: the written world is the cell's (the runtime binding's world
    SHA-256 equals the recorded cell world), the record names its own cell, only lit_rendered equals S4's world, the
    four cells of a scene come from one S4 world (and lit_rendered's is it), and the stack binding (without the
    world) is one across all four cells."""
    p, cores = [], {}
    by_env = collections.defaultdict(list)
    for r in rows:
        if not r.get("valid"):
            continue
        w = r.get("cell_world") or {}
        by_env[r.get("env")].append(r)
        if w.get("cell") != r.get("arm"):
            p.append("episode %s: cell world %r is not its cell %r" % (r.get("index"), w.get("cell"), r.get("arm")))
        bw = _bound_world(r)
        if bw is None or bw != w.get("world_sha256"):
            p.append("episode %s: the bound world SHA-256 is not the cell world's" % r.get("index"))
        if (w.get("equals_s4_world") is True) != (r.get("arm") == "lit_rendered"):
            p.append("episode %s: equals_s4_world %r for cell %s" % (r.get("index"), w.get("equals_s4_world"),
                                                                       r.get("arm")))
        cores.setdefault(_core(r), []).append(r["arm"])
    if len(cores) > 1:
        p.append("the stack binding differs across cells: %s" % [sorted(set(v)) for v in cores.values()])
    for env, rs in sorted(by_env.items(), key=lambda kv: str(kv[0])):
        bases = {(r.get("cell_world") or {}).get("s4_world_sha256") for r in rs}
        lr = [(r.get("cell_world") or {}).get("world_sha256") for r in rs if r.get("arm") == "lit_rendered"]
        if len(bases) != 1 or None in bases:
            p.append("scene %s: the cells come from %d different S4 worlds" % (env, len(bases)))
        elif lr and lr[0] not in bases:
            p.append("scene %s: lit_rendered's world is not the scene's S4 world" % env)
    return p


def ulog_summary(rows, attempt_dir=None):
    """Totals, and with attempt_dir every recorded .gz decompressed and matched to its record (review r1 LOW-5)."""
    out = {"episodes": 0, "without_ulog": 0, "not_all_verified": 0, "files": 0, "bytes": 0, "gz_bytes": 0,
           "file_problems": None}
    probs = []
    for r in rows:
        u = r.get("ulog")
        if r.get("outcome") in NOT_RUN_OUTCOMES:
            continue
        out["episodes"] += 1
        if not u or not u.get("count"):
            out["without_ulog"] += 1
            continue
        if not u.get("all_verified"):
            out["not_all_verified"] += 1
        for f in u["files"]:
            out["files"] += 1
            out["bytes"] += f.get("bytes") or 0
            out["gz_bytes"] += f.get("gz_bytes") or 0
            if attempt_dir is not None and f.get("verified"):
                probs += ["ep%03d %s" % (r["index"], x)
                          for x in verify_ulog_record(pathlib.Path(attempt_dir) / ("ep%03d" % r["index"]), f)]
    if attempt_dir is not None:
        out["file_problems"] = probs
    return out


def manipulation_checks(rows, attempt_dir=None):
    """Per cell, from each VALID episode's sensor_steps.npy (S4's columns: 5 colour-rule px, 6 shadow px, 11 LiDAR and
    12 depth returns removed as ground): removed counts per step and at hover (step 0), and the colour-rule to shadow
    pixel ratio on steps with >= 2 shadow pixels (review r1 MINOR-7). Descriptive."""
    import numpy as np
    out = {}
    for cell in CELL_NAMES:
        steps, hover, missing = [], [], 0
        for r in rows:
            if r.get("arm") != cell or not r.get("valid"):
                continue
            f = pathlib.Path(attempt_dir) / ("ep%03d" % r["index"]) / "sensor_steps.npy" if attempt_dir else None
            if f is None or not f.is_file():
                missing += 1
                continue
            S = np.load(f)
            if len(S):
                steps.append(S)
                hover.append(S[0])
        if not steps:
            out[cell] = {"episodes": 0, "missing": missing}
            continue
        A, H = np.concatenate(steps), np.array(hover)
        vis = A[:, 6] >= 2
        st = lambda x: {"median": float(np.median(x)), "max": float(x.max())}            # noqa: E731
        out[cell] = {"episodes": len(steps), "missing": missing, "steps": int(len(A)),
                     "lidar_ground_removed_per_step": st(A[:, 11]), "depth_ground_removed_per_step": st(A[:, 12]),
                     "hover_step0": {"lidar": st(H[:, 11]), "depth": st(H[:, 12]),
                                     "episodes_lidar_removed_gt0": int((H[:, 11] > 0).sum())},
                     "colour_to_shadow_px_ratio_median": float(np.median(A[vis, 5] / A[vis, 6])) if vis.any() else None,
                     "shadow_visible_steps": int(vis.sum())}
    return out


def holm(pvals):
    """Holm step-down adjusted p-values, in the input order."""
    m, adj, run = len(pvals), [None] * len(pvals), 0.0
    for rank, i in enumerate(sorted(range(m), key=lambda j: pvals[j])):
        run = max(run, min(1.0, (m - rank) * pvals[i]))
        adj[i] = run
    return adj


def primary_contrasts(paired, label):
    """The two preregistered primary contrasts, main A and main G on capture, Holm over the two at 0.05 (review r1
    MINOR-2). `reached` is read only when the reading is REPORTED; every other quantity is descriptive."""
    rows = [dict(outcome=o, contrast=k, **{x: paired[o][k][x] for x in ("mean_pp", "scenes_positive",
                                                                          "scenes_negative", "scenes_zero")},
                 p_raw=paired[o][k]["sign_test_p_two_sided"]) for o, k in PRIMARY]
    for r, a in zip(rows, holm([r["p_raw"] for r in rows])):
        r["p_holm"] = a
        r["reached"] = (a <= PRIMARY_ALPHA) if label == "REPORTED" else None
    return {"alpha": PRIMARY_ALPHA, "method": "exact two-sided sign test, Holm over the two primary contrasts",
            "contrasts": rows}


def reading_label(attempt, final, integ, complete):
    """VOID: the attempt is not COMPLETE (ABORTED, termination) or the kernel-fault count rose or an integrity check
    failed; INSUFFICIENT: fewer than MIN_COMPLETE_SCENES scenes VALID in all four cells; else REPORTED."""
    faults = final.get("kernel_faults_after") != (attempt or {}).get("kernel_faults_before")
    if final.get("status") != "COMPLETE" or faults or integ:
        why = ("attempt %s: %s" % (final.get("status"), final.get("reason")) if final.get("status") != "COMPLETE"
               else "kernel faults %r -> %r" % ((attempt or {}).get("kernel_faults_before"),
                                                final.get("kernel_faults_after")) if faults else "; ".join(integ))
        return "VOID", why
    if complete < MIN_COMPLETE_SCENES:
        return "INSUFFICIENT", "%d of %d scenes VALID in all four cells (< %d)" % (complete, N_SCENES,
                                                                                  MIN_COMPLETE_SCENES)
    return "REPORTED", None


def reading_from(attempt, final, rows, isaac, s3b, s3b_ref, s4rows, s4_ref, envs=None, attempt_dir=None,
                 world_models_now=None):
    """The fixed reading from the attempt's records. Raises ValueError on malformed rows (scene_table). A VOID
    reading carries no outcome numbers (review r1 MINOR-3)."""
    if attempt is not None and attempt.get("development_smoke"):
        raise ValueError("a development smoke has no reading")
    integ = S3C.arm_integrity_problems(attempt, final, rows)
    if attempt is not None:
        if attempt.get("protocol") != PROTOCOL:
            integ.append("attempt protocol %r is not %s" % (attempt.get("protocol"), PROTOCOL))
        integ += design_problems(attempt.get("schedule", []), envs)
        integ += world_model_problems(attempt.get("world_models"))
    integ += record_problems(rows)
    paired = {k: paired_effects(rows, k) for k in ("capture", "crash")}       # validates the rows first
    label, why = reading_label(attempt, final, integ, paired["capture"]["scenes_valid_in_all_cells"])
    out = {"protocol": PROTOCOL, "attempt_status": final.get("status"), "attempt_reason": final.get("reason"),
           "label": label, "label_reason": why, "integrity_problems": integ,
           "invalid_per_cell": {c: dict(collections.Counter(r.get("invalid_reason") for r in rows
                                                           if r["arm"] == c and not r.get("valid")))
                                for c in CELL_NAMES}}
    if label == "VOID":
        return out
    out.update({"s3b_reference": s3b_ref, "s4_reference": s4_ref, "cells": {}, "paired": paired,
                "primary": primary_contrasts(paired, label),
                "replication": replication(rows, s4rows, s4_ref, attempt, world_models_now),
                "timing_and_state": SF.timing_summary([dict(r, block=r["arm"]) for r in rows]),
                "ulog": ulog_summary(rows, attempt_dir), "manipulation_checks": manipulation_checks(rows, attempt_dir),
                "host_gpu": {c: sum(1 for r in rows if r["arm"] == c and r.get("outcome") in ("GPU_BUSY",
                                                                                              "GPU_UNCHECKED"))
                             for c in CELL_NAMES}})
    for cell in CELL_NAMES:
        ar = [r for r in rows if r["arm"] == cell]
        out["cells"][cell] = cell_summary(ar, isaac, s3b) if ar else None
    lr = [r for r in rows if r["arm"] == "lit_rendered"]
    out["lit_rendered_vs_s4_ground"] = vs_s4_ground(lr, s4rows)
    return out


def write_once(path, text):
    """Refuses to replace an existing file with other bytes; an identical rewrite is a no-op."""
    path = pathlib.Path(path)
    if path.exists():
        if path.read_text() != text:
            raise SystemExit("refused: %s exists with other content" % path)
        return False
    path.write_text(text)
    return True


def analyze(attempt_dir=ATTEMPT_DIR, s3b_attempt=SF.ATTEMPT_DIR, s4_ledger=S4_LEDGER, isaac=None,
            world_models_now=None, envs=None):
    attempt_dir = pathlib.Path(attempt_dir)
    attempt, final, rows = S3C.load_attempt(attempt_dir)            # refuses a development smoke
    isaac = SF.isaac_outcomes(SF.G8, BARS) if isaac is None else isaac
    s3b, s3b_ref = s3b_reference(s3b_attempt)
    s4rows, s4_ref = s4_reference(s4_ledger)
    if envs is None:
        try:
            envs = scene_envs()
        except OSError:
            envs = None
    if world_models_now is None:
        try:
            world_models_now = world_model_files()
        except Exception:                                            # noqa: BLE001 - reported as not compared
            world_models_now = None
    try:
        out = reading_from(attempt, final, rows, isaac, s3b, s3b_ref, s4rows, s4_ref, envs, attempt_dir,
                           world_models_now)
    except ValueError as e:
        raise SystemExit("refused: %s" % e)
    write_once(attempt_dir / "reading.json", json.dumps(out, indent=1, sort_keys=True, default=str) + "\n")
    print(json.dumps({"status": out["attempt_status"], "label": out["label"],
                      "integrity": out["integrity_problems"]}), flush=True)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--smoke", choices=CELL_NAMES)
    g.add_argument("--launch", action="store_true")
    g.add_argument("--analyze", nargs="?", const=str(ATTEMPT_DIR))
    a = ap.parse_args(argv)
    if a.check:
        check()
    elif a.smoke:
        launch(smoke_cell=a.smoke)
    elif a.launch:
        launch()
    else:
        analyze(a.analyze)


if __name__ == "__main__":
    main()
