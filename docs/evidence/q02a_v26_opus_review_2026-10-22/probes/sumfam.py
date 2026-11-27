import json, glob, sys
for p in sorted(glob.glob(sys.argv[1] + "/*.json")):
    j = json.load(open(p)); c = j["counts"]
    print("%-12s seed %2d scenes %5d fit %-90s offsetOK %3d spurious_no_true %3d WORST_FALSE %3d" % (j["family"], j["seed"], c["scenes"],
          {k[4:]: v for k, v in c.items() if k.startswith("fit_")}, c.get("ok_offset", 0), c.get("ok_spurious_no_true_inlier", 0), c.get("WORST_FALSE_SUPPORT_SCENES", 0)))
