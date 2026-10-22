import json, sys
mine = json.load(open(sys.argv[1]))["counts"]; ref = json.load(open(sys.argv[2]))[sys.argv[3]]["v26"]
KEYS = ("scenes", "scenes_with_material_false", "tracks", "supported", "FALSE_SUPPORT", "FALSE_MATERIAL", "FALSE_GAP_LE_5CM", "FALSE_CENTRE_IN", "correct_tracks_with_truth", "correct_supported", "INCORRECT_REJECTION")
bad = 0
for k in KEYS:
    a = mine.get(k, 0); b = ref["sweep"].get(k, 0); bad += a != b; print("%-28s mine %9d ref %9d %s" % (k, a, b, "" if a == b else "DIFF"))
for k in KEYS[2:]:
    a = mine.get("TL_" + k, 0); b = ref["tracker_like"].get(k, 0); bad += a != b; print("TL_%-25s mine %9d ref %9d %s" % (k, a, b, "" if a == b else "DIFF"))
fit = {k[4:]: v for k, v in mine.items() if k.startswith("fit_")}; print("fit", fit == ref["fit"], fit); print("crosschecks", mine.get("crosschecks"), ref["crosschecks"]); print("MISMATCHES", bad)
