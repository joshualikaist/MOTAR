"""CPU-only interface/geometry probes. No runtime observer, planner or action sink.

The header is a proposed logging contract; tests use explicit synthetic metadata.
No missing production timestamp, calibration or mission loss is fabricated here.
"""
import argparse
from dataclasses import dataclass, replace
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import torch
from motar_eval.boia.access import read_online
from motar_eval.boia.types import OnlineInputs
from motar_eval.boia_research.decision import decide, initial_state
from motar_eval.boia_research.types import ResearchCase, SwitchPolicy

_spec = importlib.util.spec_from_file_location("r5_access_fixtures", ROOT / "tests/test_boia_access.py")
fixtures = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fixtures)


@dataclass(frozen=True)
class Header:
    env: str
    episode: str
    decision: int
    phase: str
    source_sha: str  # Git source commit (40 hex); config_sha is a SHA-256 digest.
    config_sha: str
    lineage: str
    decision_sim_s: float
    camera_capture_sim_s: object = None
    camera_delivery_sim_s: object = None
    lidar_capture_sim_s: object = None
    lidar_delivery_sim_s: object = None
    host_arrival_monotonic_s: object = None
    camera_frame: object = None
    lidar_frame: object = None
    depth_convention: object = None
    capture_pose_bound: bool = False
    raw_lidar_validity_preserved: bool = False
    detect_decoupled: bool = False
    layout: tuple = (4, 72)
    camera_shape: tuple = (90, 160)
    max_age_s: object = None  # Logging policy, not measured physical uncertainty.


def finite(x):
    try:
        return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
    except (OverflowError, TypeError):
        return False


def sha_id(value, width=64):
    return isinstance(value, str) and len(value) == width and all(c in "0123456789abcdef" for c in value)


def audit_header(packet, header):
    """Validate declared schema only; permanently abstain from actual mission claims.

    Does not consume evaluator truth, modify F or issue any executable proposal.
    Schema success is not a runtime timestamp/pose/parity qualification.
    """
    if not isinstance(packet, OnlineInputs) or not isinstance(header, Header):
        raise TypeError("only copied OnlineInputs and explicit Header records are permitted")
    reasons = []
    if (not isinstance(header.env, str) or not header.env.strip()
            or not isinstance(header.episode, str) or not header.episode.strip()
            or not isinstance(header.decision, int)
            or isinstance(header.decision, bool) or header.decision < 0):
        reasons.append("INVALID_EPISODE_DECISION_ID")
    if header.phase != "DECISION_PRE_ACTION":
        reasons.append("NOT_CURRENT_DECISION_PHASE")
    if not sha_id(header.source_sha, 40) or not sha_id(header.config_sha) or not header.lineage:
        reasons.append("SOURCE_CONFIG_LINEAGE_UNBOUND")
    if header.lineage != "FROZEN_F_OFF":
        reasons.append("OTHER_LINEAGE_NOT_QUALIFIED_BY_THIS_SPEC")
    stamps = (header.camera_capture_sim_s, header.camera_delivery_sim_s,
              header.lidar_capture_sim_s, header.lidar_delivery_sim_s)
    if (not all(finite(x) and x >= 0 for x in (*stamps, header.decision_sim_s, header.max_age_s))
            or not finite(header.host_arrival_monotonic_s) or header.host_arrival_monotonic_s < 0):
        reasons.append("CLOCKS_OR_AGE_POLICY_MISSING")
    elif (not stamps[0] <= stamps[1] <= header.decision_sim_s
          or not stamps[2] <= stamps[3] <= header.decision_sim_s
          or any(header.decision_sim_s - x > header.max_age_s for x in (stamps[0], stamps[2]))):
        reasons.append("STALE_OR_MIXED_SIM_CLOCKS")
    if (header.camera_frame != "YAW_VEHICLE_CAMERA_RAY" or header.lidar_frame != "YAW_LIDAR_RAY"
            or header.depth_convention != "EUCLIDEAN_RAY_RANGE_M" or header.capture_pose_bound is not True):
        reasons.append("FRAME_DEPTH_OR_CAPTURE_POSE_UNBOUND")
    if header.raw_lidar_validity_preserved is not True:
        reasons.append("SANITIZED_LIDAR_LOST_RETURN_PROVENANCE")
    if header.layout != (4, 72) or tuple(packet.lidar_m.shape[1:]) != (288,):
        reasons.append("UNSUPPORTED_LIDAR_LAYOUT")
    if header.detect_decoupled is not False:
        reasons.append("DETECT_CHANNEL_NOT_PERMITTED_BY_THIS_SPEC")
    if header.camera_shape != (90, 160):
        reasons.append("UNSUPPORTED_CAMERA_DOMAIN")
    # No capability in this probe accepts a calibration tag or a surrogate gain
    # as real mission evidence. Missing model safety is not a physical emergency.
    return {"schema_valid_under_declarations": not reasons, "schema_reasons": tuple(sorted(reasons)),
            "utility_outcome": "ABSTAIN", "utility_reason": "ACTUAL_MISSION_MODEL_UNAVAILABLE",
            "hypothesis_status": "UNKNOWN_OR_UNOBSERVED", "safety_status": "UNAVAILABLE",
            "actual_emergency": "NOT_ASSESSED", "control_owner": "F_UNCHANGED_EXTERNALLY",
            "research_selector": "NOT_EVALUATED", "runtime_qualified": False}


def lidar_kind(raw_normalized):
    if not finite(raw_normalized) or not 0 <= raw_normalized <= 1:
        return "INVALID_OR_CLIPPED"
    if raw_normalized == 1:
        return "FAR_OR_NO_RETURN"
    if raw_normalized == 0:
        return "NEAR_OR_CLIPPED"
    return "FINITE_FIRST_HIT_NO_ID"


BRANCH_GROUPS = (
    "source", "config", "checkpoint", "actor_obs", "actor_rms", "history", "previous_action",
    "own_state", "target_executor", "obstacle_geometry", "controller_motor", "governor_feedback",
    "tracker_latency", "exogenous_rng", "terminal_phase",
)


def compare_branch_descriptors(left, right):
    """Fingerprint equality is necessary, not a simulator restore/causal proof."""
    if any(not sha_id(p.get(k)) for p in (left, right) for k in BRANCH_GROUPS):
        return "MISSING_REQUIRED_STATE_FINGERPRINT"
    if any(left[k] != right[k] for k in BRANCH_GROUPS):
        return "PREFIX_DESCRIPTOR_MISMATCH"
    return "MATCHED_DECLARATIONS_ONLY_FULL_RESTORE_NOT_QUALIFIED"


def example_header(**changes):
    # Synthetic CPU schema fixture, NOT metadata exported by current production.
    return replace(Header("env0", "episode0", 7, "DECISION_PRE_ACTION", "a" * 40,
                          "b" * 64, "FROZEN_F_OFF", .7, .7, .7, .7, .7, 10.,
                          "YAW_VEHICLE_CAMERA_RAY", "YAW_LIDAR_RAY", "EUCLIDEAN_RAY_RANGE_M",
                          True, True, False, (4, 72), (90, 160), .2), **changes)


class Probe(unittest.TestCase):
    def setUp(self):
        self.task, self.perception, self.action, self.params = fixtures.fake_inputs(2)
        self.packet = read_online(self.task, self.perception, self.action, self.params)

    def test01_missing_clocks_never_become_now(self):
        h = example_header(camera_capture_sim_s=None)
        out = audit_header(self.packet, h)
        self.assertFalse(out["schema_valid_under_declarations"])
        self.assertEqual(out["utility_outcome"], "ABSTAIN")

    def test02_frame_depth_and_stale_mismatch(self):
        for h in (example_header(depth_convention="OPTICAL_Z_M"), example_header(camera_frame="BODY_FULL_ATTITUDE"),
                  example_header(camera_capture_sim_s=.1), example_header(camera_delivery_sim_s=10.)):
            self.assertFalse(audit_header(self.packet, h)["schema_valid_under_declarations"])

    def test03_reset_phase_never_joined_to_old_episode(self):
        for phase in ("POST_RESET", "PRE_TERMINAL"):
            self.assertIn("NOT_CURRENT_DECISION_PHASE", audit_header(self.packet, example_header(phase=phase))["schema_reasons"])

    def test04_source_layout_and_lineage_binding(self):
        for changes in (dict(source_sha="unknown"), dict(source_sha="a" * 64), dict(config_sha="b" * 40),
                        dict(layout=(4, 36)), dict(lineage="S_L1C_V2"),
                        dict(detect_decoupled=True), dict(camera_shape=(270, 480))):
            self.assertFalse(audit_header(self.packet, example_header(**changes))["schema_valid_under_declarations"])

    def test05_declared_complete_header_still_abstains(self):
        out = audit_header(self.packet, example_header())
        self.assertTrue(out["schema_valid_under_declarations"])
        self.assertEqual((out["utility_outcome"], out["safety_status"], out["actual_emergency"]),
                         ("ABSTAIN", "UNAVAILABLE", "NOT_ASSESSED"))
        self.assertFalse(out["runtime_qualified"])

    def test06_observer_copy_cannot_mutate_F_input_or_output(self):
        saved = self.action.clone()
        observed = read_online(self.task, self.perception, self.action, self.params)
        observed.policy_action.fill_(99.)
        self.assertTrue(torch.equal(self.action, saved))
        self.assertEqual(observed.policy_action.data_ptr() == self.action.data_ptr(), False)

    def test07_privileged_values_do_not_change_permitted_copy(self):
        before = self.packet
        self.task.obs_dict["navrl_target_position"].fill_(1.e6)
        self.task.detector.target_mask.fill_(0)
        self.task.task_obs["states"].fill_(-999.)
        after = read_online(self.task, self.perception, self.action, self.params)
        fixtures.assert_inputs_equal(self, before, after)

    def test08_repeated_reader_preserves_rng_inputs_and_history(self):
        rng, trng = random.getstate(), torch.get_rng_state().clone()
        first = read_online(self.task, self.perception, self.action, self.params)
        second = read_online(self.task, self.perception, self.action, self.params)
        fixtures.assert_inputs_equal(self, first, second)
        self.assertEqual(random.getstate(), rng)
        self.assertTrue(torch.equal(torch.get_rng_state(), trng))

    def test09_far_invalid_cannot_be_observed_free(self):
        self.assertEqual(lidar_kind(1.), "FAR_OR_NO_RETURN")
        for raw in (float("nan"), float("inf"), -1.):
            self.assertEqual(lidar_kind(raw), "INVALID_OR_CLIPPED")
        self.assertEqual(lidar_kind(5.8 / 12.), "FINITE_FIRST_HIT_NO_ID")
        self.assertIn("SANITIZED_LIDAR_LOST_RETURN_PROVENANCE",
                      audit_header(self.packet, example_header(raw_lidar_validity_preserved=False))["schema_reasons"])

    def test10_B03_between_ray_counterexample(self):
        # Two zero-width 0/5deg rays both miss a radius.1 disk centred at6m,2.5deg.
        perpendicular = 6. * math.sin(math.radians(2.5))
        self.assertGreater(perpendicular, .1)
        self.assertAlmostEqual(2 * 6 * math.sin(math.radians(2.5)), .523432648384032, places=12)
        # The disk can lie behind the forward camera; no global 3D free certificate.

    def test11_B04_actual_LiDAR_sphere_and_foreground_same_firsthit(self):
        sphere = 6. - .20
        foreground = 5.8
        self.assertAlmostEqual(sphere, foreground, places=14)
        camera_origin_to_center = math.hypot(6. - .1, .03)
        camera_surface = camera_origin_to_center - .15
        self.assertNotAlmostEqual(camera_surface, sphere, places=3)
        # Same LiDAR value is not a proof of identity or identical entire RGB-D scenes.

    def test12_seed_only_not_complete_continuation(self):
        self.assertEqual(compare_branch_descriptors({"seed": 4701}, {"seed": 4701}),
                         "MISSING_REQUIRED_STATE_FINGERPRINT")
        a = {k: "0" * 64 for k in BRANCH_GROUPS}
        self.assertEqual(compare_branch_descriptors(a, a), "MATCHED_DECLARATIONS_ONLY_FULL_RESTORE_NOT_QUALIFIED")
        for key in ("history", "controller_motor", "exogenous_rng", "terminal_phase"):
            self.assertEqual(compare_branch_descriptors(a, dict(a, **{key: "1" * 64})), "PREFIX_DESCRIPTOR_MISMATCH")

    def test13_contact_capture_alias_semantics(self):
        # Literal independent truth table for the source mask; no historical rate inferred.
        contact, captured = True, True
        resolved_crash = contact and not captured
        self.assertFalse(resolved_crash)
        self.assertTrue(contact and captured)
        self.assertFalse(captured and not contact)  # contact-free capture is a separate event.

    def test14_shadow_abstention_does_not_relabel_selector_or_F_safety(self):
        c = ResearchCase(("env0", "episode0", 7), "F", True, (), None, (0.,), (), SwitchPolicy(.2, .1))
        model = decide(c, initial_state("env0", "episode0", "F"))
        self.assertEqual(model.outcome, "NO_CERTIFIED_BACKUP")
        out = audit_header(self.packet, example_header())
        self.assertEqual((out["utility_outcome"], out["research_selector"], out["actual_emergency"]),
                         ("ABSTAIN", "NOT_EVALUATED", "NOT_ASSESSED"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Probe))
    paths = [ROOT / "motar_eval/boia/access.py", ROOT / "tests/test_boia_access.py", Path(__file__).resolve()]
    receipt = {"schema": "motar.g4r5.cpu-probe.v1", "status": "PASS" if result.wasSuccessful() else "FAIL",
               "ran": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
               "names": [name for name in dir(Probe) if name.startswith("test")],
               "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
               "production_sensor_or_simulator_imported": any(x == "aerial_gym" or x.startswith("aerial_gym.") for x in sys.modules),
               "actual_sensor_packets": "NOT_TESTED", "GPU": "NOT_RUN", "mission_gain": "NOT_TESTED",
               "runtime_shadow_parity": "NOT_TESTED", "G5": "HOLD"}
    with open(args.output, "x") as stream:
        json.dump(receipt, stream, indent=2, allow_nan=False)
        stream.write("\n")
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == "__main__":
    main()
