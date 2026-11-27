"""Observation-weighted time shares of states (from per-episode counters), post-first-fused-acquisition approx.
fused-invalid share after acquisition = (obs - first_fused + 1 - obs_fused_valid)/(obs - first_fused + 1).
Geometric shares use all observations of acquired episodes: out_of_fov = 1-geo_in_fov/obs; centre_occluded_in_fov=(geo_in_fov-geo_visible)/obs."""
import json, glob, os, collections, sys
R = '/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results'
C = [('OCC-P2_static', R + '/occlusion_confirmatory_v1/OCC-P2/static/A1/*/kroc2027_episodes.jsonl', 4),
     ('OCC-P2_E2', R + '/occlusion_confirmatory_v1/OCC-P2/moving/A3/*/kroc2027_episodes.jsonl', 4),
     ('KROC-A1', R + '/kroc2027_A_v1/A1/*/kroc2027_episodes.jsonl', 16),
     ('KROC-A3', R + '/mt_p1_precision_v1/KROC27-A-M02/E2_direct_l2_v1/A3/*/kroc2027_episodes.jsonl', 16)]
for name, pat, q in C:
    agg = collections.defaultdict(collections.Counter)
    for f in sorted(glob.glob(pat)):
        b = int(os.path.basename(os.path.dirname(f)).split('__')[1].replace('bars', ''))
        for line in open(f):
            d = json.loads(line)
            if d['env_episode_index'] >= q or not d['ever_fused_valid']: continue
            A = agg[b]; obs = d['observations']; ff = d['first_obs_fused_valid']
            post = obs - ff + 1
            A['post'] += post; A['post_invalid'] += post - d['obs_fused_valid']
            A['obs'] += obs; A['in_fov'] += d['obs_geo_in_fov']; A['vis'] += d['obs_geo_visible']
            A['geo_vis_undetected'] += d['geo_visible_undetected_obs']
    for b in sorted(agg):
        A = agg[b]
        print(name, b, f"fused_invalid_post_acq={A['post_invalid']/A['post']:.4f} out_of_fov={1-A['in_fov']/A['obs']:.4f} centre_occ_in_fov={(A['in_fov']-A['vis'])/A['obs']:.4f} geo_visible_but_camera_invalid={A['geo_vis_undetected']/A['obs']:.4f} obs={A['obs']}")
