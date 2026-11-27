"""Outcome of the full estimate for the correct track in the random scenes that lost support; and the one spurious scene."""
import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from fast import *
cnt = collections.Counter(); sp = []
for s in (1, 2, 3):
    d = json.load(open(sys.argv[1] % s))
    for sc_ in d["scenes"]:
        if not sc_["sweep"].get("LOST_correct") and sc_["winner"] != "spurious": continue
        az, el = math.radians(sc_["az"]), math.radians(sc_["el"])
        dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]); tv = CAM_O + sc_["R"] * dr
        boxes = [tuple(b) for b in sc_["boxes"]]
        depth, lidar, paste = scene(tv, boxes)
        for cov in (0.0025, 0.01, 0.04):
            inp = T.make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar.reshape(-1), cov_pos=cov, yaw=sc_["yaw"])
            r = ev.estimate(inp)
            cnt[(sc_["winner"], r.identity_status, r.reasons[1] if len(r.reasons) > 1 else r.reasons[0])] += 1
        if sc_["winner"] == "spurious":
            sp.append(dict(seed=s, R=round(sc_["R"], 2), az=round(sc_["az"], 1), el=round(sc_["el"], 1), boxes=[[round(x, 2) for x in b] for b in boxes], paste=sc_["paste"], cand=sc_["cand"], nin=sc_["nin"]))
for k, v in sorted(cnt.items()): print(k, v)
print("spurious scenes:", sp)
