"""Scene-hash overlap between OCC-P2 cells and KROC-B cells (same seed numbers 4301-4307), and OCC-P1/KROC-A. Read-only."""
import json, glob, os, collections
R = '/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results'
def hashes(pat, q):
    s = collections.defaultdict(set)
    for f in glob.glob(pat):
        cell = os.path.basename(os.path.dirname(f))
        for line in open(f):
            d = json.loads(line)
            if d['env_episode_index'] < q:
                s[cell.split('__', 1)[1]].add(d['initial']['scene_hash'])
    return s
p2 = hashes(R + '/occlusion_confirmatory_v1/OCC-P2/static/A1/*/kroc2027_episodes.jsonl', 4)
p2m = hashes(R + '/occlusion_confirmatory_v1/OCC-P2/moving/A3/*/kroc2027_episodes.jsonl', 4)
b0 = hashes(R + '/kroc2027_B_v1/B0/*/kroc2027_episodes.jsonl', 16)
a1 = hashes(R + '/kroc2027_A_v1/A1/*/kroc2027_episodes.jsonl', 4)
p1 = hashes(R + '/occlusion_pilot_v1/OCC-P1_rerun1/static/A1/*/kroc2027_episodes.jsonl', 4)
tot = inter = 0
for k in sorted(p2):
    if k in b0:
        i = len(p2[k] & b0[k]); tot += len(p2[k]); inter += i
print('OCC-P2 static vs KROC-B0 (same seed/density cells):', inter, 'of', tot, 'P2 scene hashes also in B0')
allb = set().union(*b0.values()); allp2 = set().union(*p2.values()) | set().union(*p2m.values())
print('any-cell overlap P2(static+moving) vs B0:', len(allp2 & allb), 'of', len(allp2))
print('static vs moving P2 shared scene hashes:', len(set().union(*p2.values()) & set().union(*p2m.values())))
allp1 = set().union(*p1.values()) if p1 else set(); alla1 = set().union(*a1.values())
print('OCC-P1 static (n=%d) vs KROC-A1 first-4 (n=%d) overlap:' % (len(allp1), len(alla1)), len(allp1 & alla1))
