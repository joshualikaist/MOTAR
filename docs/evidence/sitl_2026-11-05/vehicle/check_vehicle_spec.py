"""CPU pre-verification of the SITL vehicle definitions (no Gazebo or PX4 needed).

Parses a Gazebo SDF (iris or the MOTAR training-spec navrl_quad draft) and a PX4 airframe file, and computes from the
files themselves:
  - total mass;
  - inertia about the centre of mass, rotor links included by the parallel-axis theorem;
  - rotor positions;
  - max thrust per rotor (motorConstant x maxRotVelocity^2) and thrust-to-weight;
  - yaw moment ratio;
  - spin directions.
It checks that the SDF (FLU) and the PX4 control allocation (FRD) describe the same rotors with matching spin signs,
and, for navrl_quad, that everything matches MOTAR's Isaac Gym URDF and robot config.
usage: check_vehicle_spec.py OUT_JSON
"""
import json
import math
import pathlib
import re
import sys
import xml.dom.minidom

BASE = pathlib.Path(__file__).resolve().parents[1]
MOTAR = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
G = 9.81


def txt(node, tag):
    e = node.getElementsByTagName(tag)
    return e[0].firstChild.nodeValue.strip() if e and e[0].firstChild else None


def sdf_spec(path):
    doc = xml.dom.minidom.parse(str(path))
    links = {}
    for link in doc.getElementsByTagName("link"):
        name = link.getAttribute("name")
        pose = [float(v) for v in (txt(link, "pose") or "0 0 0 0 0 0").split()]
        inert = link.getElementsByTagName("inertial")
        if not inert:
            continue
        m = float(txt(inert[0], "mass"))
        I = [float(txt(inert[0], k)) for k in ("ixx", "iyy", "izz")]
        links[name] = {"pos": pose[:3], "mass": m, "I": I}
    motors = []
    for p in doc.getElementsByTagName("plugin"):
        if p.getAttribute("filename") == "libgazebo_motor_model.so":
            motors.append({k: txt(p, k) for k in ("linkName", "turningDirection", "timeConstantUp", "timeConstantDown",
                                                 "maxRotVelocity", "motorConstant", "momentConstant", "motorNumber",
                                                 "rotorDragCoefficient", "rollingMomentCoefficient")})
    mass = sum(l["mass"] for l in links.values())
    com = [sum(l["mass"] * l["pos"][i] for l in links.values()) / mass for i in range(3)]
    Ixx = sum(l["I"][0] + l["mass"] * ((l["pos"][1] - com[1]) ** 2 + (l["pos"][2] - com[2]) ** 2) for l in links.values())
    Iyy = sum(l["I"][1] + l["mass"] * ((l["pos"][0] - com[0]) ** 2 + (l["pos"][2] - com[2]) ** 2) for l in links.values())
    Izz = sum(l["I"][2] + l["mass"] * ((l["pos"][0] - com[0]) ** 2 + (l["pos"][1] - com[1]) ** 2) for l in links.values())
    rotors = []
    for mm in sorted(motors, key=lambda x: int(x["motorNumber"])):
        pos = links[mm["linkName"]]["pos"]
        tmax = float(mm["motorConstant"]) * float(mm["maxRotVelocity"]) ** 2
        rotors.append({"n": int(mm["motorNumber"]), "x_flu": pos[0], "y_flu": pos[1], "dir": mm["turningDirection"],
                       "tmax_N": tmax, "km": float(mm["momentConstant"]), "tau_up": float(mm["timeConstantUp"]),
                       "tau_down": float(mm["timeConstantDown"]), "rotor_drag": float(mm["rotorDragCoefficient"]),
                       "rolling_moment": float(mm["rollingMomentCoefficient"])})
    T = sum(r["tmax_N"] for r in rotors)
    arm = max(abs(r["y_flu"]) for r in rotors)
    return {"mass_kg": mass, "inertia_com": [Ixx, Iyy, Izz], "rotors": rotors, "total_max_thrust_N": T,
            "thrust_to_weight": T / (mass * G),
            "max_horizontal_accel_at_full_thrust_mps2": G * math.sqrt(max(0.0, (T / (mass * G)) ** 2 - 1)),
            "roll_accel_authority_radps2": 2 * max(r["tmax_N"] for r in rotors) * arm / Ixx,
            "yaw_accel_authority_radps2": sum(r["tmax_N"] * r["km"] for r in rotors) / Izz}


def px4_rotors(path):
    vals = dict(re.findall(r"param set-default (CA_ROTOR\d_(?:PX|PY|KM))\s+(\S+)", pathlib.Path(path).read_text()))
    n = int(re.search(r"param set-default CA_ROTOR_COUNT\s+(\d+)", pathlib.Path(path).read_text()).group(1))
    return [{"n": i, "px_frd": float(vals["CA_ROTOR%d_PX" % i]), "py_frd": float(vals["CA_ROTOR%d_PY" % i]),
             "km": float(vals["CA_ROTOR%d_KM" % i])} for i in range(n)]


def consistency(sdf, px4, tol_pos=0.035):
    """Same rotor index: FRD (px, py) = FLU (x, -y) within tol_pos; ccw <-> KM > 0 (PX4 convention, as in iris)."""
    out = []
    for r, p in zip(sdf["rotors"], px4):
        out.append({"n": r["n"], "pos_match": abs(r["x_flu"] - p["px_frd"]) <= tol_pos and abs(-r["y_flu"] - p["py_frd"]) <= tol_pos,
                    "pos_err_m": [r["x_flu"] - p["px_frd"], -r["y_flu"] - p["py_frd"]],
                    "spin_match": (r["dir"] == "ccw") == (p["km"] > 0), "km_match_sdf": abs(abs(p["km"]) - r["km"]) < 1e-9})
    return out


def isaac_spec():
    urdf = MOTAR / "resources/robots/quad/quad_navrl_collide.urdf"
    doc = xml.dom.minidom.parse(str(urdf))
    mass = sum(float(m.getAttribute("value")) for m in doc.getElementsByTagName("mass"))
    base_I = doc.getElementsByTagName("inertia")[0]
    cfg = (MOTAR / "aerial_gym/config/robot_config/base_quad_config.py").read_text()
    tmax = float(re.search(r"max_thrust\s*=\s*([\d.]+)", cfg).group(1))
    km = float(re.search(r"thrust_to_torque_ratio\s*=\s*([\d.]+)", cfg).group(1))
    tau = float(re.search(r"motor_time_constant_increasing_max\s*=\s*([\d.]+)", cfg).group(1))
    motor_mass, arm = 0.00625, 0.13
    Ixx = float(base_I.getAttribute("ixx")) + 4 * motor_mass * arm ** 2
    Izz = float(base_I.getAttribute("izz")) + 4 * motor_mass * 2 * arm ** 2
    return {"urdf": str(urdf), "mass_kg": mass, "inertia_com_approx": [Ixx, Ixx, Izz], "max_thrust_per_motor_N": tmax,
            "thrust_to_weight": 4 * tmax / (mass * G), "thrust_to_torque": km, "motor_tau_s": tau, "arm_xy_m": arm}


def main():
    out = pathlib.Path(sys.argv[1])
    iris = sdf_spec(BASE / "reference/iris.sdf.jinja")
    nq = sdf_spec(BASE / "models/navrl_quad/navrl_quad.sdf.jinja")
    isaac = isaac_spec()
    res = {"iris": iris, "navrl_quad_sdf": nq, "isaac_navrl_quad": isaac,
           "iris_px4_consistency": consistency(iris, px4_rotors(BASE / "reference/px4_v1.14.3_10015_gazebo-classic_iris"), 0.035),
           "navrl_px4_consistency": consistency(nq, px4_rotors(BASE / "airframes/10090_gazebo-classic_navrl_quad"), 1e-9)}
    chk = {
        "navrl_mass_matches_isaac": abs(nq["mass_kg"] - isaac["mass_kg"]) < 1e-6,
        "navrl_tmax_matches_isaac": all(abs(r["tmax_N"] - isaac["max_thrust_per_motor_N"]) < 1e-9 for r in nq["rotors"]),
        "navrl_km_matches_isaac": all(abs(r["km"] - isaac["thrust_to_torque"]) < 1e-12 for r in nq["rotors"]),
        "navrl_tau_matches_isaac": all(r["tau_up"] == r["tau_down"] == isaac["motor_tau_s"] for r in nq["rotors"]),
        "navrl_tw_matches_isaac": abs(nq["thrust_to_weight"] - isaac["thrust_to_weight"]) < 1e-6,
        "navrl_inertia_within_2pct_of_isaac": all(abs(a - b) / b < 0.02 for a, b in zip(nq["inertia_com"], isaac["inertia_com_approx"])),
        "navrl_no_rotor_drag": all(r["rotor_drag"] == 0 and r["rolling_moment"] == 0 for r in nq["rotors"]),
        "navrl_sdf_px4_rotors_consistent": all(c["pos_match"] and c["spin_match"] and c["km_match_sdf"] for c in res["navrl_px4_consistency"]),
        "iris_sdf_px4_spin_consistent_reference": all(c["spin_match"] for c in res["iris_px4_consistency"]),
    }
    res["checks"] = chk
    res["pass"] = all(chk.values())
    res["ratios_iris_over_navrl"] = {"mass": iris["mass_kg"] / nq["mass_kg"],
                                     "Ixx": iris["inertia_com"][0] / nq["inertia_com"][0],
                                     "thrust_to_weight": iris["thrust_to_weight"] / nq["thrust_to_weight"],
                                     "roll_authority": iris["roll_accel_authority_radps2"] / nq["roll_accel_authority_radps2"],
                                     "yaw_authority": iris["yaw_accel_authority_radps2"] / nq["yaw_accel_authority_radps2"]}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1)
        fh.write("\n")
    print(json.dumps({"pass": res["pass"], "checks": chk, "ratios_iris_over_navrl": res["ratios_iris_over_navrl"],
                      "iris": {k: iris[k] for k in ("mass_kg", "inertia_com", "thrust_to_weight",
                                                    "max_horizontal_accel_at_full_thrust_mps2",
                                                    "roll_accel_authority_radps2", "yaw_accel_authority_radps2")},
                      "navrl_quad_sdf": {k: nq[k] for k in ("mass_kg", "inertia_com", "thrust_to_weight",
                                                           "max_horizontal_accel_at_full_thrust_mps2",
                                                           "roll_accel_authority_radps2", "yaw_accel_authority_radps2")},
                      "isaac": isaac, "iris_px4_pos_err": [c["pos_err_m"] for c in res["iris_px4_consistency"]]},
                     indent=1))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
