"""S3a: does the bridge rebuild Policy F's observation and action exactly? (docs/plans/sitl_validation_plan_v1.md)

Input: a G8 run (tools/sitl_g8_ref_dump.py): per cell the observation dump (NAVRL_OBS_DUMP, stride 1), the state record
(tools/staterec_hook.py) and the occlusion trace (bar layouts). For the first N envs of a cell, each env's first
episode is replayed from its start through tools/sitl_bridge.py (MOTAR's own sensors, perception and policy on the CPU)
with the recorded states, and compared step by step with Isaac's dumped observation and the recorded action:
  obs_err[slice]   max |bridge obs - dumped obs| per slice: scan 0-287, obstacles 288-767, robot 768-817, target 818-897
  act_err_bridge   max |policy(bridge obs) - recorded action|
  act_err_dumpobs  max |policy(dumped obs) - recorded action|   (isolates the policy's CPU vs GPU numerics)
  gov_err          max |bridge prev_action from step k (governor on the bridge's LiDAR, recorded action) -
                   recorded prev_action at step k + 1|   (the governed command path)
Gate (plan S3a): EQUIVALENT if every obs_err <= 1e-3, act_err_bridge <= 1e-4 and gov_err <= 1e-4; otherwise NOT_EQUIVALENT, with the
first diverging step and slice reported. Alignment checks: the dump's ep_step equals the record's sim_steps at the
same call index, and episode 0 of an env is the run of calls from 0 while sim_steps counts up.
usage: tools/sitl_s3a_equivalence.py G8_RUN_DIR BARS OUT_JSON [N_ENVS]
"""
import hashlib
import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

RUN, BARS, OUT = pathlib.Path(sys.argv[1]).resolve(), int(sys.argv[2]), pathlib.Path(sys.argv[3])
N_ENVS = int(sys.argv[4]) if len(sys.argv) > 4 else 8
SEED = 4501
SLICES = {"scan": (0, 288), "obstacles": (288, 768), "robot": (768, 818), "target": (818, 898)}
OBS_TOL, ACT_TOL = 1e-3, 1e-4


def main():
    if OUT.exists():
        raise SystemExit("refusing to overwrite " + str(OUT))
    import torch
    import sitl_bridge as B
    cid = "E2_obstacle_aware__%dbars__seed%d" % (BARS, SEED)
    cell = RUN / "full" / cid
    receipt = json.loads((cell / "receipt.json").read_text())
    M = B.setup(receipt["env_snapshot_navrl"])
    ot_spec = __import__("importlib.util").util.spec_from_file_location("ot_s3a", str(B.ROOT / "motar_eval/occlusion_trace.py"))
    OT = __import__("importlib.util").util.module_from_spec(ot_spec)
    ot_spec.loader.exec_module(OT)
    dump = np.load(RUN / ("obs_dump_%dbars_seed%d.npz" % (BARS, SEED)))
    rec = np.load(RUN / ("state_record_%dbars_seed%d.npz" % (BARS, SEED)))
    tr = OT.read_trace(cell)
    look = OT.reset_lookup(tr["layouts"])
    policy = B.Policy()
    d_obs, d_env, d_call, d_step = dump["obs"], dump["env_id"], dump["call_index"], dump["ep_step"]
    index = {(int(c), int(e)): i for i, (c, e) in enumerate(zip(d_call, d_env))}
    steps = rec["sim_steps"]                                            # [calls, envs]
    out = {"analysis": "S3a bridge equivalence", "cell": cid, "inputs_sha256": {}, "envs": {}, "tolerances":
           {"obs": OBS_TOL, "action": ACT_TOL}, "alignment_mismatches": 0}
    for p in (cell / "receipt.json", RUN / ("obs_dump_%dbars_seed%d.npz" % (BARS, SEED)),
              RUN / ("state_record_%dbars_seed%d.npz" % (BARS, SEED)), cell / "occlusion_layouts.npz"):
        out["inputs_sha256"][p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    worst = {k: 0.0 for k in SLICES}
    worst_act_b, worst_act_d, worst_gov = 0.0, 0.0, 0.0
    first_div = None
    for e in range(N_ENVS):
        ks = []
        for k in range(steps.shape[0]):
            if steps[k, e] != k:
                break
            ks.append(k)
        c, h, q = OT.layout_arrays(tr["layouts"], look[(e, 0)])
        ep = B.Episode(c, h, q)
        env_worst = {k: 0.0 for k in SLICES}
        ab, ad, ge = 0.0, 0.0, 0.0
        for k in ks:
            i = index.get((k, e))
            if i is None or int(d_step[i]) != int(steps[k, e]):
                out["alignment_mismatches"] += 1
                continue
            s = {"pos": rec["robot_position"][k, e], "vel": rec["robot_linvel"][k, e],
                 "vquat": rec["robot_vehicle_orientation"][k, e], "fquat": rec["robot_orientation"][k, e],
                 "angvel": rec["robot_body_angvel"][k, e], "tpos": rec["target_position"][k, e],
                 "tquat": rec["target_orientation"][k, e] if "target_orientation" in rec.files else None,
                 "prev_action": rec["prev_action"][k, e], "bounds_min": rec["env_bounds_min"][k, e],
                 "bounds_max": rec["env_bounds_max"][k, e]}
            obs, _, lidar = ep.observe(s, step_count=int(rec["perception_step_count"][k]) - 1)
            ref = torch.from_numpy(d_obs[i]).reshape(1, -1)
            diff = (obs - ref).abs()[0]
            for name, (a, b) in SLICES.items():
                v = float(diff[a:b].max())
                env_worst[name] = max(env_worst[name], v)
                if first_div is None and v > OBS_TOL:
                    first_div = {"env": e, "call": k, "slice": name, "max_err": v,
                                 "argmax": int(a + int(diff[a:b].argmax()))}
            act_rec = torch.from_numpy(rec["action"][k, e]).reshape(1, -1)
            ab = max(ab, float((policy.act(obs) - act_rec).abs().max()))
            ad = max(ad, float((policy.act(ref) - act_rec).abs().max()))
            if k + 1 in ks:
                _, _, prev_next = ep.command(rec["action"][k, e], lidar)
                ge = max(ge, float((prev_next - torch.from_numpy(rec["prev_action"][k + 1, e])).abs().max()))
        out["envs"][str(e)] = {"steps": len(ks), "obs_err": env_worst, "act_err_bridge": ab, "act_err_dumpobs": ad,
                               "gov_err": ge}
        for name in SLICES:
            worst[name] = max(worst[name], env_worst[name])
        worst_act_b, worst_act_d = max(worst_act_b, ab), max(worst_act_d, ad)
        worst_gov = max(worst_gov, ge)
        print("env", e, len(ks), {k: "%.2e" % v for k, v in env_worst.items()}, "act %.2e / %.2e gov %.2e" % (ab, ad, ge),
              flush=True)
    out["worst"] = {"obs_err": worst, "act_err_bridge": worst_act_b, "act_err_dumpobs": worst_act_d, "gov_err": worst_gov}
    out["first_divergence"] = first_div
    ok = (all(v <= OBS_TOL for v in worst.values()) and worst_act_b <= ACT_TOL and worst_gov <= ACT_TOL
          and out["alignment_mismatches"] == 0)
    out["reading"] = "EQUIVALENT" if ok else "NOT_EQUIVALENT"
    OUT.write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"reading": out["reading"], "worst": out["worst"], "first_divergence": first_div,
                      "alignment_mismatches": out["alignment_mismatches"]}, indent=1))


if __name__ == "__main__":
    main()
