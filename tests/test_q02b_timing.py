"""Q02-B CPU tests: eligibility, loss events, local map, candidates, timing features, shadow machine.

Synthetic ray-cast episodes reuse the Q02-A scene helper. They check contracts and the declared
rules against known geometry; they do not show capture benefit or timing optimality.
"""
import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q02_evidence as q2a
from motar_eval.q02 import belief as bl, candidates as cd, contracts as c, evidence as ev, localmap as lm
from motar_eval.q02 import shadow as sh, timing as tm
from motar_eval.q02.history import HistoryError, ORACLE_PROXY, ObservationHistory, loss_state
from motar_eval.q02.pipeline import ShadowRecorder

BAR = ((4., 0., .3),)


def frame(k, **kw):
    clock = c.Clock(0.1 * k, 0.1 * k, "DECLARED_SIM_SCHEDULE")
    kw.setdefault("cam_clock", clock); kw.setdefault("lidar_clock", clock)
    return q2a.make(decision=0.1 * k, **kw)


def run(script, policy="BOIA", episode=1, terminal_at=None):
    """script: list of per-decision kwargs for q2a.make; returns ShadowDecisions."""
    rec = ShadowRecorder(policy); rec.reset(episode); out = []
    for k, kw in enumerate(script):
        term = "capture" if terminal_at == k else None
        out.append(rec.observe(episode, k, frame(k, **kw), ORACLE_PROXY, terminal=term)[0])
    return out


SEEN = dict(detected=True)
OCC = dict(bars=BAR)


class EligibilityAndLossTests(unittest.TestCase):
    def test_01_never_acquired_abstains_out_of_scope(self):
        out = run([dict(render_target=False)] * 6)
        self.assertTrue(all(d.decision == sh.ABSTAIN and d.reasons == ("NOT_PREVIOUSLY_ACQUIRED",) for d in out))
        self.assertEqual(out[-1].state_after, sh.PRE_ACQUISITION)
        self.assertEqual(out[-1].features[0].value, False)

    def test_02_normal_tracking_continues_F(self):
        out = run([SEEN] * 8)
        self.assertEqual(out[0].reasons, ("NOT_PREVIOUSLY_ACQUIRED",))  # one detection is not an acquisition
        self.assertTrue(all(d.decision == sh.CONTINUE_F and d.state_after == sh.TRACK_F for d in out[1:]))
        self.assertEqual(dict((f.name, f.value) for f in out[-1].features)["detector_provenance"]["realism"],
                         "NOT_REALISTIC_ORACLE_PROXY")

    def test_03_one_decision_occlusion_is_debounced(self):
        out = run([SEEN] * 4 + [OCC] + [SEEN] * 3)
        self.assertNotIn(sh.LOSS_SUSPECTED, [d.state_after for d in out])

    def test_04_long_occlusion_suspects_loss_and_BOIA_abstains(self):
        out = run([SEEN] * 4 + [OCC] * 20)
        self.assertEqual(out[-1].state_after, sh.LOSS_SUSPECTED)
        self.assertTrue(all(d.decision == sh.ABSTAIN for d in out[5:]))
        self.assertIn("UTILITY_NOT_ESTIMATED", out[-1].reasons)
        self.assertEqual(out[-1].delta_u.status, tm.NOT_ESTIMATED)
        self.assertNotIn(sh.PROPOSE_REACQUIRE_SHADOW, [d.decision for d in out])

    def test_04b_baseline_triggers_propose_only_in_shadow(self):
        for policy in ("H1", "H3"):
            with self.subTest(policy=policy):
                out = run([SEEN] * 4 + [OCC] * 20, policy=policy)
                props = [d for d in out if d.decision == sh.PROPOSE_REACQUIRE_SHADOW]
                self.assertTrue(props); self.assertIn("EXECUTOR_NOT_AVAILABLE", props[0].reasons)
                self.assertTrue(all(d.owner == "F" and d.action is None and not d.action_applied for d in out))
        f0 = run([SEEN] * 4 + [OCC] * 20, policy="F0")
        self.assertNotIn(sh.PROPOSE_REACQUIRE_SHADOW, [d.decision for d in f0])

    def test_05_leaving_the_field_of_view(self):
        out = run([SEEN] * 4 + [dict(target_v=(-6., 2., 0.))] * 6)
        self.assertEqual(out[-1].evidence_hypothesis, ev.OUT_OF_FOV)
        self.assertEqual(dict((f.name, f.value) for f in out[-1].features)["fov_status"], "OUTSIDE")
        self.assertEqual(out[-1].state_after, sh.LOSS_SUSPECTED)

    def test_06_detector_miss_counts_as_miss(self):
        out = run([SEEN] * 4 + [dict(render_target=False)] * 4)
        self.assertEqual(out[-1].evidence_hypothesis, ev.DETECTION_MISS)
        self.assertEqual(out[-1].state_after, sh.LOSS_SUSPECTED)

    def test_07_08_false_lock_jump_is_identity_suspect_even_with_high_confidence(self):
        jump = [dict(detected=True, confidence=0.99, target_v=(8., 0., 0.) if k % 2 == 0 else (8., 3., 0.)) for k in range(8)]
        out = run(jump)
        self.assertTrue(all(d.reasons == ("NOT_PREVIOUSLY_ACQUIRED",) for d in out))  # never a valid acquisition
        out2 = run([SEEN] * 4 + [dict(detected=True, target_v=(8., 3., 0.))] + [OCC] * 8)
        self.assertIn("IDENTITY_SUSPECT", out2[-1].reasons)

    def test_09_growing_covariance_blocks_triggers(self):
        script = [SEEN] * 4 + [dict(bars=BAR, cov_pos=0.01 + 0.25 * k) for k in range(12)]
        out = run(script, policy="H2")
        self.assertIn("EVIDENCE_UNKNOWN", out[-1].reasons)
        self.assertNotEqual(out[-1].decision, sh.PROPOSE_REACQUIRE_SHADOW)

    def test_10_missing_sensor_is_not_a_target_loss(self):
        out = run([SEEN] * 4 + [dict(cam_clock=c.Clock(None, None))] * 6 + [SEEN])
        self.assertTrue(all(d.reasons == ("INPUT_INVALID",) for d in out[4:10]))
        self.assertNotIn(sh.LOSS_SUSPECTED, [d.state_after for d in out])

    def test_11_decision_clock_inconsistency_refused(self):
        rec = ShadowRecorder(); rec.reset(1)
        for k in range(3):
            rec.observe(1, k, frame(k, **SEEN), ORACLE_PROXY)
        bad = q2a.make(decision=0.55, cam_clock=c.Clock(.55, .55, "DECLARED_SIM_SCHEDULE"),
                       lidar_clock=c.Clock(.55, .55, "DECLARED_SIM_SCHEDULE"), detected=True)
        d, _ = rec.observe(1, 3, bad, ORACLE_PROXY)
        self.assertEqual(d.reasons, ("INPUT_INVALID",))
        stale = q2a.make(decision=0.3, cam_clock=c.Clock(0.1, 0.1, "DECLARED_SIM_SCHEDULE"), detected=True)
        self.assertIn("camera_clock:STALE", " ".join(ev.estimate(stale).validation_problems))

    def test_12_depth_lidar_disagreement_is_reported(self):
        inp = frame(0, bars=BAR)
        lidar = np.array(inp.lidar); lidar[2, 36] = 7.8  # return at the target's expected range on the axis
        inp2 = c.EvidenceInput.build(**{**{f: getattr(inp, f) for f in inp.__dataclass_fields__}, "lidar": lidar.reshape(-1)})
        r = ev.estimate(inp2)
        # Q02A-v2.6 (was the v1 reason LIDAR_RETURN_ALSO_AT_EXPECTED_RANGE): the camera foreground decides, and the
        # LiDAR return at the expected range is reported as per-sensor evidence in the timing features.
        self.assertEqual(r.hypothesis, ev.OCCLUDED); self.assertGreater(r.lidar.consistent, 0)
        feats, _ = tm.extract(inp2, r, loss_state(ObservationHistory()), None, (), None, 0)
        occ = dict((f.name, f.value) for f in feats)["occlusion_evidence"]
        self.assertGreater(occ["lidar"]["consistent"], 0); self.assertGreater(occ["camera"]["foreground"], 0)

    def test_17_no_second_prediction_of_a_covered_interval(self):
        snap = bl.from_tracker(frame(0, **SEEN))
        self.assertIs(bl.forecast(snap, snap.time_s), snap)
        with self.assertRaisesRegex(ValueError, "PAST"):
            bl.forecast(snap, snap.time_s - 0.1)
        three = bl.forecast(snap, round(snap.time_s + 0.3, 9))
        f, q = np.eye(6), np.diag([.025 * .01] * 3 + [.025] * 3)
        f[0, 3] = f[1, 4] = f[2, 5] = .1
        x, p = np.array(snap.mean), np.array(snap.cov)
        for _ in range(3):
            x, p = f @ x, f @ p @ f.T + q
        self.assertTrue(np.allclose(three.mean, x) and np.allclose(three.cov, p))
        self.assertIs(bl.forecast(three, three.time_s), three)
        with self.assertRaisesRegex(ValueError, "MULTIPLE"):
            bl.forecast(snap, snap.time_s + 0.05)

    def test_history_contract_errors(self):
        h = ObservationHistory()
        inp = frame(0, **SEEN); rec = ev.estimate(inp)
        with self.assertRaises(HistoryError):
            h.append(1, 0, inp, rec, ORACLE_PROXY)  # before reset
        h.reset(1)
        with self.assertRaisesRegex(HistoryError, "PROVENANCE"):
            h.append(1, 0, inp, rec, "SIMULATOR_TRUTH")
        h.append(1, 0, inp, rec, ORACLE_PROXY)
        with self.assertRaisesRegex(HistoryError, "CONSECUTIVE"):
            h.append(1, 2, frame(2, **SEEN), rec, ORACLE_PROXY)
        with self.assertRaisesRegex(HistoryError, "INCREASING"):
            h.reset(1)


class MapAndCandidateTests(unittest.TestCase):
    def test_numpy_known_free_equals_boia_geometry(self):
        import torch
        from motar_eval.boia.geometry import known_free
        rng = np.random.default_rng(7)  # local generator; the global numpy stream is untouched
        for _ in range(5):
            scan = rng.uniform(0.2, 12.0, (4, 72)).astype(np.float32)
            ours = lm.known_free_radii(scan, c.FROZEN_F_LIDAR, 0.45)
            ref = known_free(torch.tensor(scan.reshape(1, -1)), torch.tensor(c.FROZEN_F_LIDAR.azimuth_rad()),
                             torch.zeros(1, 3), torch.zeros(1), 12.0, 0.45)
            self.assertTrue(np.allclose(ours, ref.numpy()[0], atol=1e-5))

    def map_for(self, bars=BAR, **kw):
        inp = frame(0, bars=bars, **kw)
        return lm.build(inp, c.FROZEN_F_LIDAR), inp

    def test_13_14_blocked_and_unknown_paths_are_never_safe(self):
        m, inp = self.map_for()
        self.assertEqual(lm.segment_status(m, (0., 0.), (1.5, 0.), 0.45)[0], lm.SAFE)
        self.assertEqual(lm.segment_status(m, (0., 0.), (4., 0.), 0.45)[0], lm.BLOCKED)
        status, unknown, why = lm.segment_status(m, (0., 0.), (6., 0.6), 0.2)
        self.assertNotEqual(status, lm.SAFE)
        status, unknown, why = lm.segment_status(m, (0., 0.), (0., 1.), 0.45, now_s=0.5)
        self.assertEqual((status, why), (lm.UNCERTIFIED, ("MAP_STALE",)))

    def test_15_no_feasible_candidate_returns_NO_FEASIBLE_PLAN(self):
        ring = tuple((0.9 * math.cos(a), 0.9 * math.sin(a), .25) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False))
        m, inp = self.map_for(bars=ring)
        snap = bl.from_tracker(inp)
        recs, plan, _ = cd.evaluate(m, m.origin_xy, 0.0, snap, cd.generate(m.origin_xy, 0.0), 0.0)
        self.assertEqual(plan, cd.NO_FEASIBLE_PLAN)
        self.assertFalse(any(r.status == cd.FEASIBLE for r in recs))
        out = run([SEEN] * 4 + [dict(bars=ring)] * 8, policy="H3")
        self.assertIn("NO_FEASIBLE_PLAN", out[-1].reasons); self.assertEqual(out[-1].decision, sh.ABSTAIN)
        self.assertEqual(out[-1].fallback_if_no_plan, "UNDEFINED_PENDING_CONTROLLER_QUALIFICATION")

    def test_candidate_features_behind_an_occluder(self):
        m, inp = self.map_for()
        recs, plan, _ = cd.evaluate(m, m.origin_xy, 0.0, bl.from_tracker(inp), cd.generate(m.origin_xy, 0.0), 0.0)
        self.assertIsNone(plan)
        ahead = [r for r in recs if r.kind.startswith("VIEWPOINT_b+00")]
        self.assertTrue(all(r.view_from_current == "BLOCKED" for r in recs))
        feas = [r for r in recs if r.status == cd.FEASIBLE]
        self.assertTrue(feas and all(r.visibility_gain >= 1 and r.handover_feasible for r in feas))
        self.assertTrue(all(r.path_unknown_fraction == 0.0 for r in feas))


class StateMachineTests(unittest.TestCase):
    def test_transition_table_and_unreachable_execution(self):
        for a, b in ((sh.PRE_ACQUISITION, sh.LOSS_SUSPECTED), (sh.TRACK_F, sh.REACQUIRE_PLANNED),
                     (sh.TRACK_F, sh.REACQUIRE_EXECUTING), (sh.TERMINAL_CAPTURE, sh.TRACK_F)):
            with self.subTest(a=a, b=b), self.assertRaises(sh.TransitionError):
                sh.validate_transition(a, b)
        sh.validate_transition(sh.LOSS_SUSPECTED, sh.TERMINAL_FAILURE)
        for policy in sh.POLICIES:
            out = run([SEEN] * 4 + [OCC] * 30, policy=policy)
            self.assertNotIn(sh.REACQUIRE_EXECUTING, [d.state_after for d in out])

    def test_16_reacquisition_then_loss_cannot_chatter(self):
        script = [SEEN] * 4 + [OCC] * 14 + [SEEN] * 3 + [OCC] * 14 + [SEEN] * 3 + [OCC] * 14
        out = run(script, policy="H3")
        entries = [i for i in range(1, len(out)) if out[i].state_after == sh.REACQUIRE_PLANNED
                   and out[i - 1].state_after != sh.REACQUIRE_PLANNED]
        gaps = [out[b].decision_s - out[a].decision_s for a, b in zip(entries, entries[1:])]
        self.assertTrue(all(g >= sh.MachineParams().cooldown_s for g in gaps), gaps)
        self.assertIn("COOLDOWN_ACTIVE", [r for d in out for r in d.reasons])
        self.assertLessEqual(len(entries), sh.MachineParams().max_proposals_per_episode)

    def test_terminal_is_absorbing_and_evaluator_sourced(self):
        out = run([SEEN] * 5, terminal_at=4)
        self.assertEqual((out[-1].state_after, out[-1].terminal_source), (sh.TERMINAL_CAPTURE, "EVALUATOR_EPISODE_END"))
        rec = ShadowRecorder(); rec.reset(1)
        rec.observe(1, 0, frame(0, **SEEN), ORACLE_PROXY, terminal="failure")
        with self.assertRaises(sh.TransitionError):
            rec.observe(1, 1, frame(1, **SEEN), ORACLE_PROXY)


class FirewallTests(unittest.TestCase):
    def test_18_ground_truth_cannot_be_passed_in(self):
        rec = ShadowRecorder(); rec.reset(1)
        with self.assertRaises(TypeError):
            rec.observe(1, 0, frame(0), ORACLE_PROXY, target_position=np.zeros(3))
        with self.assertRaises(sh.TransitionError):
            sh.ShadowDecision(1, 0, 0., "BOIA", sh.TRACK_F, sh.TRACK_F, sh.CONTINUE_F, (), (), None, ev.TARGET_VISIBLE,
                              None, action=(0., 0., 0., 0.))
        with self.assertRaises(sh.TransitionError):
            sh.ShadowDecision(1, 0, 0., "BOIA", sh.LOSS_SUSPECTED, sh.REACQUIRE_PLANNED, sh.PROPOSE_REACQUIRE_SHADOW,
                              (), (), None, ev.OCCLUDED, None)
        d = run([SEEN] * 3)[-1]
        self.assertEqual(tuple(f.name for f in d.features), tm.FEATURES)
        text = " ".join("%s %s" % (f.name, f.provenance) for f in d.features).upper()
        self.assertFalse(any(k in text for k in ("SEGMENTATION", "SEMANTIC", "GROUND_TRUTH", "NAVRL_TARGET", "RENDERER_POSE")))
        self.assertEqual(dict((f.name, f.provenance) for f in d.features)["relative_target_position"],
                         "TRACKER_MEAN_IN_YAW_VEHICLE_FRAME")  # the tracker estimate, not simulator truth

    def test_19_nonfinite_inputs_fail_closed(self):
        out = run([SEEN] * 4 + [dict(track_mean=np.full(6, np.nan))] * 3)
        self.assertTrue(all(d.reasons == ("INPUT_INVALID",) for d in out[4:]))
        self.assertNotIn(sh.LOSS_SUSPECTED, [d.state_after for d in out])  # invalid input is not a loss
        out = run([SEEN] * 4 + [dict(quat=np.array([.1, 0., 0., math.sqrt(.99)]))])
        self.assertEqual(out[-1].reasons, ("INPUT_INVALID",))

    def test_20_frozen_F_noninterference_contract(self):
        for path in sorted((ROOT / "motar_eval/q02").glob("*.py")):
            tree = ast.parse(path.read_text())
            mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
            mods |= {(n.module or "").split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 0}
            rel = {(n.module or "") for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level >= 1}
            with self.subTest(path=path.name):
                self.assertFalse(mods & {"torch", "isaacgym", "aerial_gym", "warp", "random", "rl_games"})
                self.assertFalse(any("boia_shadow" in r for r in rel))
                self.assertNotIn("np.random", path.read_text())
        state = np.random.get_state()[1].copy()
        out = run([SEEN] * 4 + [OCC] * 8, policy="H3")
        self.assertTrue(np.array_equal(state, np.random.get_state()[1]))
        self.assertTrue(all(d.owner == "F" and d.action is None for d in out))

    def test_q01_frames_replay_read_only_if_present(self):
        import json, os
        run_root = Path("/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/S1_ON")
        if not run_root.is_dir():
            self.skipTest("Q01-V2 raw not present on this machine")
        before = {p.name: os.stat(p).st_mtime_ns for p in run_root.iterdir()}
        rows = [json.loads(l) for l in (run_root / "sensors.jsonl").read_text().splitlines()]
        rec = ShadowRecorder()
        ck = lambda t: c.Clock(t["capture_sim_s"], t["delivery_sim_s"], t["provenance"], t["frame"], t["depth_convention"])
        for i in range(3):  # snapshots are 5 decisions apart: one single-decision episode each, real clocks
            z = dict(np.load(run_root / ("sensor_%03d.npz" % i), allow_pickle=False))
            inp = c.EvidenceInput.from_arrays(z, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]),
                                              rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
            rec.reset(i + 1)
            d, _ = rec.observe(i + 1, 0, inp, ORACLE_PROXY)
            self.assertEqual(d.reasons, ("NOT_PREVIOUSLY_ACQUIRED",))  # one detection is not an acquisition
            self.assertIn(d.decision, (sh.CONTINUE_F, sh.ABSTAIN)); self.assertIsNone(d.action)
        self.assertEqual(before, {p.name: os.stat(p).st_mtime_ns for p in run_root.iterdir()})


class IdentityContractIntegrationTests(unittest.TestCase):
    """Q02-B against the Q02A-v2.6 identity contract (detection / geometry / identity verdict / provenance)."""

    def test_provenance_must_agree_with_the_evidence_record(self):
        h = ObservationHistory(); h.reset(1)
        inp = frame(0, **SEEN); rec = ev.estimate(inp)
        from motar_eval.q02.history import ERROR_INJECTED, IMAGE_DETECTOR
        for lineage in (ERROR_INJECTED, IMAGE_DETECTOR):      # no record contract exists yet for these lineages
            with self.subTest(lineage=lineage), self.assertRaisesRegex(HistoryError, "PROVENANCE_MISMATCH"):
                h.append(1, 0, inp, rec, lineage)
        e = h.append(1, 0, inp, rec, ORACLE_PROXY)
        self.assertEqual(e.record_provenance, "ORACLE_TRUE_POSE_RASTER")

    def test_acquisition_is_provisional_and_never_an_identity_confirmation(self):
        out = run([SEEN] * 4)
        ident = dict((f.name, f.value) for f in out[-1].features)["identity_evidence"]
        self.assertEqual(ident["acquisition_kind"], "PROVISIONAL_FOOTPRINT_CONSISTENT_NOT_IDENTITY_CONFIRMED")
        self.assertEqual(ident["identity_verdict"], "CONSISTENT_NOT_CONFIRMED")
        self.assertEqual(ident["identity_confirmation"], "NOT_AVAILABLE_SINGLE_FRAME_ORACLE_FLAG")
        self.assertEqual((out[-1].identity_verdict, out[-1].detector_provenance), ("CONSISTENT_NOT_CONFIRMED", "ORACLE_TRUE_POSE_RASTER"))

    def test_ambiguous_or_untestable_detections_never_acquire_or_reacquire(self):
        box = dict(target_v=(4.0, 0.2, 0.0), detected=True, depth=q2a.box_target_scene((4.0, 0.2, 0.0)))   # G2: AMBIGUOUS
        stale = dict(detected=True, age=5.0)                                                              # NOT_TESTABLE
        for name, kw in (("ambiguous", box), ("untestable", stale)):
            with self.subTest(case=name):
                out = run([kw] * 6)
                self.assertTrue(all(d.reasons == ("NOT_PREVIOUSLY_ACQUIRED",) for d in out))   # never acquired
        out = run([SEEN] * 4 + [OCC] * 8 + [box] * 4, policy="H3")
        self.assertEqual(out[-1].state_after, sh.LOSS_SUSPECTED)          # a wrong-shape detection is not a reacquisition
        self.assertIn("IDENTITY_SUSPECT", out[-1].reasons)
        out = run([SEEN] * 4 + [OCC] * 8 + [SEEN] * 3, policy="H3")
        self.assertEqual(out[-1].state_after, sh.TRACK_F)
        ident = dict((f.name, f.value) for f in out[-1].features)["identity_evidence"]
        self.assertEqual(ident["acquisition_kind"], "PROVISIONAL_FOOTPRINT_CONSISTENT_NOT_IDENTITY_CONFIRMED")

    def test_review_H1_H2_adversarial_fixtures_states_are_recorded(self):
        """Opus v2.6 review residuals: what the history makes of them (documented limits, not hidden)."""
        cam_o = np.array(q2a.CAM.camera_offset)
        # H1-class residual (corner Z2 R 3, d_f 0.4, a=b=-0.85): a blend sphere wins; a track on it is footprint-consistent.
        tv, dr, depth, lidar = q2a._front_corner_scene(3.0, 0.4, -0.85, -0.85)
        rep = q2a.va.validate(q2a.make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar), q2a.CAM, q2a.LID)
        fit = q2a.geo.fit_target_sphere(depth, rep.depth_pasted, q2a.CAM, q2a.CAM.target_radius)
        self.assertGreater(np.linalg.norm(fit.centre - tv), 0.2)          # the accepted sphere is a blend sphere
        h = ObservationHistory(); h.reset(1)
        inp = q2a.make(target_v=tuple(fit.centre), detected=True, depth=depth, lidar=lidar)   # wrong track on it
        e = h.append(1, 0, inp, ev.estimate(inp), ORACLE_PROXY)
        # KNOWN LIMIT (review H1, P2): the residual false footprint support reaches Q02-B as a PROVISIONAL hit.
        # It is labelled CONSISTENT_NOT_CONFIRMED / PROVISIONAL and is never an identity confirmation.
        self.assertTrue(e.usable_detection); self.assertEqual(e.identity_verdict, "CONSISTENT_NOT_CONFIRMED")
        # H2 (P2, fail-safe): a correct track on a 15 m target fits too few inliers -> not a hit, no acquisition.
        far = q2a.make(target_v=(15.0, 0.3, 0.0), detected=True)
        rf = ev.estimate(far)
        self.assertEqual((rf.sphere_fit_status, rf.identity_verdict), ("TOO_FEW_INLIERS", "NOT_TESTABLE"))
        h2 = ObservationHistory(); h2.reset(1)
        self.assertFalse(h2.append(1, 0, far, rf, ORACLE_PROXY).usable_detection)
        self.assertTrue(all(d.reasons == ("NOT_PREVIOUSLY_ACQUIRED",) for d in run([dict(target_v=(15.0, 0.3, 0.0), detected=True)] * 5)))

if __name__ == "__main__":
    unittest.main()
