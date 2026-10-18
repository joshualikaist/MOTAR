"""OCC-P2: association of terminal outcomes with active indicator intervals at episode end (descriptive only)."""
import json,sys,collections
a=json.load(open(sys.argv[1]))
agg=collections.defaultdict(lambda: collections.Counter())
for k,c in a['cells'].items():
    g=(c['group'],c['density'])
    A=agg[g]
    for o,n in c['outcomes'].items(): A['out_'+o]+=n
    A['episodes']+=c['episodes']; A['acq']+=c['acquired_episodes']
    for o,n in c['duration']['end_causes'].items(): A['center_end_'+o]+=n
    for ind in ('out_of_view','geometric_full_in_view','fused_loss','rendered_full_in_view'):
        for o,n in c['diagnostic_indicators'][ind]['end_causes'].items(): A[ind+'_end_'+o]+=n
    p=c['parity']; A['frames']+=p['frames']; A['gfull_frames']+=p['geometric_full_probe_frames']; A['clear_frames']+=p['clear_in_view_probe_frames']
    A['onset_c2b']+=c['onset_kinds'].get('clear_to_blocked',0); A['onset_reentry']+=c['onset_kinds'].get('in_view_reentry_while_already_blocked',0)
    for kk,vv in c['onset_kinds'].items():
        if kk not in ('clear_to_blocked','in_view_reentry_while_already_blocked'): A['onset_'+kk]+=vv
print('group dens eps crash timeout | crash@centreOcc crash@OOV crash@gfull crash@fusedloss | share_crash_centre share_crash_OOV share_crash_(centre+OOV) | frame share gfull, clear_in_view, other | onset c2b/reentry/other')
for g in sorted(agg,key=lambda x:(x[0],x[1])):
    A=agg[g]; cr=A['out_crash']
    cc=A['center_end_crash']; co=A['out_of_view_end_crash']; cg=A['geometric_full_in_view_end_crash']; cf=A['fused_loss_end_crash']
    fr=A['frames']
    oth={k:v for k,v in A.items() if k.startswith('onset_') and k not in ('onset_c2b','onset_reentry')}
    print(g, A['episodes'], cr, A['out_timeout'], '|', cc, co, cg, cf, '|', f"{cc/cr:.3f} {co/cr:.3f} {(cc+co)/cr:.3f}", '|', f"{A['gfull_frames']/fr:.4f} {A['clear_frames']/fr:.4f} {1-(A['gfull_frames']+A['clear_frames'])/fr:.4f}", '|', A['onset_c2b'], A['onset_reentry'], oth, 'timeout ends centre:',A['center_end_timeout'],'oov:',A['out_of_view_end_timeout'],'oov_capture',A['out_of_view_end_capture'],'acq',A['acq'])
