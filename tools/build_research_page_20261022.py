#!/usr/bin/env python3
"""Static publication page + figure atlas, built only from documentation records."""
import html
import json
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
ASSETS='../assets/paper/redesign-2026-10-22'


def read(path):return json.loads((ROOT/path).read_text())


def pp(x,n=2):return ('−' if x<0 else '+')+format(abs(x),'.'+str(n)+'f')


def files(stem):
    return '<span class="file-links">'+''.join('<a href="'+ASSETS+'/'+stem+'.'+e+'">'+e.upper()+'</a>' for e in ['svg','pdf','png'])+'</span>'


def figure(i,caption):
    f=MANIFEST['figures'][i-1];stem=f['id']
    return '<figure class="paper-figure" id="figure-'+str(i)+'"><div class="figure-surface"><a href="'+ASSETS+'/'+stem+'.svg" aria-label="Open Figure '+str(i)+' full-size SVG"><img src="'+ASSETS+'/'+stem+'.svg" alt="'+html.escape(f['caption'],quote=True)+'" width="1200" height="'+str(round(1200*f['height_in']/f['width_in']))+'" loading="'+('eager' if i==1 else 'lazy')+'"></a></div><figcaption><div class="caption-copy"><strong>Figure '+str(i)+'. '+f['title']+'.</strong> '+caption+'</div>'+files(stem)+'</figcaption></figure>'


CAPTIONS={
    1:'The baseline receives RGB-D, LiDAR and simulator own state. Flat observations precede network tokenization; the task replaces actor z with PI altitude hold. Riskcap is the Policy F baseline; arc-clearance is a separate experimental filter. Arrows show selected data dependencies. Sensor rendering is synthetic; this is not a real-image flight stack.',
    2:'The scalar dimensions sum to 898. The network projects these four groups and adds CLS to obtain 17 tokens of width 64. The five-slot history window and five-second tracker expiry are different contracts. Values describe the pinned Policy F evaluation profile.',
    3:'Seven fresh evaluation seeds; four equally weighted densities; per-environment episode quota. Open marks are seed means or paired differences; filled diamonds show the recorded BCa contrasts. The original three-seed campaign is retained separately and is not pooled.',
    4:'Exploratory re-display of the existing primary counts. Thin trajectories and open marks show individual seeds; thick marks show means. The left axis starts at 70%. Density-specific inference is not preregistered here; no new intervals or tests are claimed.',
    5:'Four separate experiments with different axes and units of replication. Open marks show recorded seed effects where available. The arc panel uses the seed mean and its seed-t interval; the historical pooled estimate is reported separately in the evidence ledger. Rendering sensitivity uses Policy R, a different checkpoint. These effects cannot be added.',
    6:'Selected dependencies of the current CPU research implementation, separate from the baseline control loop. The branch records evidence, belief, candidate screens and timing without applying actions. Q02-B/Q03 are on review HOLD; BOIA utility remains ABSTAIN. An executor and closed-loop recovery benefit are not validated.',
}


def shell(title,body,kind='',scripts=''):
    return '''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="description" content="MOTAR research: RGB-D and LiDAR observations, structured temporal policies, frozen-policy evidence, and BOIA active reacquisition research.">
<meta name="theme-color" content="#f9faf9"><title>'''+title+'''</title>
<link rel="stylesheet" href="publication.css?v=20261022-redesign"></head>
<body class="'''+kind+'''"><a class="skip-link" href="#main">Skip to content</a>
<header class="masthead"><div class="nav-inner"><a class="wordmark" href="index.html">MOTAR</a><nav aria-label="Primary">
<a href="index.html#system">System</a><a href="index.html#representation">Representation</a><a href="index.html#target-motion">Results</a><a href="index.html#boia">BOIA</a><a href="figures.html">Figures</a><a href="evidence.html">Evidence</a>
</nav></div></header>'''+body+'''
<footer class="footer"><p>MOTAR · Simulation research · October 2026</p><nav aria-label="Resources"><a href="https://github.com/joshualikaist/MOTAR-public">Public code</a><a href="https://github.com/joshualikaist/MOTAR-public/blob/main/CITATION.cff">Citation</a><a href="https://github.com/joshualikaist/MOTAR/blob/main/THIRD_PARTY_LICENSES.md">Licenses</a><a href="archive-2026-09-13.html">Historical page</a></nav></footer>'''+scripts+'''</body></html>\n'''


def head(n,title,description):
    return '<div class="chapter-head"><p class="eyebrow">'+n+'</p><div><h2>'+title+'</h2><p>'+description+'</p></div></div>'


def milestones():
    return '''<div class="table-scroll" role="region" aria-label="BOIA milestone status" tabindex="0"><table><thead><tr><th>Module</th><th>Recorded status</th><th>What this establishes</th></tr></thead><tbody>
<tr data-boia-milestone="Q01"><td>Q01-V2</td><td><span class="status">Runtime qualified</span></td><td>Bounded native/OFF/ON shadow parity for the recorded profile; not algorithm efficacy.</td></tr>
<tr data-boia-milestone="Q02-A"><td>Q02-A v2.6</td><td>CPU tested · reviewed with limits</td><td>PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS; residual P2 H1/H2.</td></tr>
<tr data-boia-milestone="Q02-B"><td>Q02-B</td><td><span class="status status-hold">Independent review FAIL · HOLD</span></td><td>Two P1 findings: loss-clock handling and current-velocity stopping checks remain unfixed.</td></tr>
<tr data-boia-milestone="Q03"><td>Q03</td><td><span class="status status-hold">Independent review FAIL · HOLD</span></td><td>Map and L1/L2 candidate screens CPU tested; no executor, L3/L4 or physical free-volume certificate.</td></tr>
<tr data-boia-milestone="Q04"><td>Q04</td><td><span class="status status-hold">Design only · D0 HOLD</span></td><td>Fixes, delta review, preregistration and separate GPU approval are required.</td></tr>
</tbody></table></div>'''


def limitations():
    return '''<p>Actual occlusion accuracy is <strong>NOT VALIDATED</strong>; persistent same-target identity is <strong>NOT CONFIRMED</strong>. The detector provenance is <code>ORACLE_TRUE_POSE_RASTER</code>, not a qualified real-image detector. The reviewed profile is spherical-target, no-distractor only. No BOIA capture improvement, online flight safety or real-flight performance has been measured. <strong>G5 HOLD.</strong></p>'''


def registries():
    reg=read('docs/research_status_registry.json')['components']
    track=read('docs/status_manifest.json')['track_d']
    out='<details><summary>Historical component and renderer registries</summary><p>Lifecycle completion and scientific verdict are different fields. These registry states describe their named historical stage, not the current BOIA gate.</p><div class="table-scroll"><table><thead><tr><th>Component</th><th>Lifecycle</th><th>Evidence verdict</th></tr></thead><tbody>'
    for key,r in reg.items():
        out+='<tr><td><a href="https://github.com/joshualikaist/MOTAR/blob/main/'+r['evidence']+'">'+html.escape(key+' · '+r['label'])+'</a></td><td data-component-id="'+key+'">'+html.escape(r['lifecycle_status'])+'</td><td data-evidence-status>'+html.escape(r['evidence_status'])+'</td></tr>'
    out+='</tbody></table></div><div class="table-scroll"><table><thead><tr><th>Renderer stage</th><th>Verdict</th></tr></thead><tbody>'
    for key,r in track.items():
        out+='<tr><td><a href="https://github.com/joshualikaist/MOTAR/blob/main/'+r['evidence']+'">'+html.escape(key+' · '+r['label'])+'</a></td><td data-status-id="'+key+'">'+html.escape(r['status'])+'</td></tr>'
    return out+'</tbody></table></div><p id="public-status-manifest" class="minor-note">Status source: <a href="../status_manifest.json">Track D manifest</a> + <a href="../research_status_registry.json">component registry</a>; lifecycle is separate from evidence verdict.</p></details>'


def build_index():
    v=MANIFEST['values']; motion=v['motion_primary']
    contrasts=[pp(100*r['mean_diff']) for r in motion]
    base=subprocess.check_output(['git','show','7689664bbaceae8c1ae6f349e41dbb6871eda3ab:docs/status/index.html'],cwd=ROOT).decode()
    demo=re.search(r'<section class="demo".*?</section>',base,re.S).group()
    # Keep the original viewer DOM and assets. Only its surrounding editorial presentation changes.
    demo=demo.replace('class="demo"','class="demo"').replace('<p class="num">Appendix</p>','<p class="eyebrow">Appendix · explanatory browser view</p>')
    scripts='\n'.join(re.findall(r'<script src="[^"]+"></script>',base))
    body='''<main id="main" class="page">
<header class="hero" id="top"><p class="eyebrow">Robotics research / simulation study</p><h1>MOTAR</h1><p class="expansion">Moving Object Tracking And Rendezvous</p>
<p class="minor-note" style="margin-top:10px">Reinforcement Learning for UAV Tracking and Close Approach in Random Obstacle Fields</p>
<p class="lead">From intermittent observations<br>to tracking and close approach.</p>
<div class="hero-bottom"><p>How do perception, temporal state and local geometry change the behavior of a frozen UAV policy in clutter? MOTAR measures those effects—and studies when lost-target tracking should give way to active reacquisition.</p><div class="resource-links"><a href="figures.html">Publication figures ↗</a><a href="evidence.html">Evidence ledger ↗</a><a href="https://github.com/joshualikaist/MOTAR-public">Public code ↗</a></div></div></header>
<div class="contract-strip"><span><strong>Sensing</strong> RGB-D + LiDAR</span><span><strong>Baseline</strong> Frozen Transformer PPO</span><span><strong>Setting</strong> Random obstacle fields</span><span><strong>Scope</strong> Simulation only</span></div>
<section class="chapter" id="system"><span id="question"></span>'''+head('01 / System','A measured loop, from sensing to motion.','A forward RGB-D camera and LiDAR support target tracking and obstacle representation. A frozen policy requests planar motion and yaw; the task retains altitude hold, and a configured speed cap precedes low-level control.')+figure(1,CAPTIONS[1])+'''
<div class="two-col"><div><h3>Target tracking and obstacle sensing overlap.</h3><p>Camera measurements initialize target tracking; LiDAR can assist association and range updates. LiDAR and depth also support obstacle features. Own state comes from the simulator, rather than a qualified hardware estimator.</p></div><div><h3>Ground truth has a defined boundary.</h3><p>Scene state generates synthetic sensor observations and supplies rewards, termination, evaluation and the critic. It is not injected directly as target pose, semantic mask or visibility into the actor observation.</p></div></div>
</section>
<section class="chapter" id="representation">'''+head('02 / Data structure','898 values first. Seventeen tokens next.','The flat actor observation is assembled outside the network. Its static scan and three histories are projected inside the Transformer; CLS is added there. The temporal representation and the tracker’s lifetime are separate contracts.')+figure(2,CAPTIONS[2])+'''
<details><summary>Read the state and feature contracts</summary><div class="table-scroll"><table><thead><tr><th>Record</th><th>Fields</th><th>Meaning</th></tr></thead><tbody>
<tr><td>Target tracker</td><td>Mean (6), covariance (6 × 6), active, age</td><td>World-frame estimated position and velocity; track activity is not confirmed identity.</td></tr>
<tr><td>Target feature (16)</td><td>Relative position (3), velocity (3), position variance (3), velocity variance (3), camera confidence, LiDAR confidence, normalized age, radius</td><td>Sensor-derived or estimated features, sampled into five history slots.</td></tr>
<tr><td>Own-state feature (10)</td><td>Vehicle-frame velocity (3), yaw rate, previous action (4), altitude, validity</td><td>Own state and previous actor action, with XY updated after the governor; not target GT.</td></tr>
<tr><td>Critic state</td><td>898 + 8 = 906</td><td>Privileged training/evaluation state; kept separate from the actor.</td></tr>
</tbody></table></div></details></section>
<section class="chapter" id="target-motion">'''+head('03 / Recorded evidence','Motion changes more than difficulty.','Against the motion lineage used during training, a static target reduced capture while obstacle-aware target motion improved the aggregate result. Seven new seeds independently replicated all three directional contrasts under a corrected per-environment quota.')+'''
<div class="stats" aria-label="Target-motion paired contrasts">'''+''.join('<div class="stat"><strong>'+contrasts[i]+' pp</strong><span>'+name+' vs H</span><small>Capture · paired seed contrast</small></div>' for i,name in enumerate(['Static E0','Constant-velocity E1','Obstacle-aware E2']))+'''</div>'''+figure(3,CAPTIONS[3])+'''
<div class="study-note"><strong>STUDY CONTRACT</strong><p>Policy F · '''+str(len(v['seeds']))+''' independent evaluation seeds · four densities · '''+format(v['episodes'],',')+''' primary episodes. pp = percentage points. All three Holm-adjusted p-values are 0.046875, at the resolution boundary of this seven-seed exact sign-flip design.</p></div>'''+figure(4,CAPTIONS[4])+'''
<p class="margin-note">The E2 gain is not uniform at high density. At 205 bars, four seed differences are positive, two negative and one zero. No target law reacts to the pursuer; obstacle-aware is not adversarial. The original first-acquisition association is consistent with a visibility mechanism; it does not identify a causal mechanism. <a href="evidence.html#replication-provenance">Design and provenance ↗</a> · <a href="https://github.com/joshualikaist/MOTAR/blob/main/docs/target_motion_algorithm_2026-09-17.md">Motion definitions</a></p></section>
<section class="chapter" id="findings">'''+head('04 / Sensitivity studies','Different interventions. Different evidence.','Measured perception error, readaptation, filter geometry and rendering were investigated in separate experiments. Their effects retain their own checkpoint, replication unit and uncertainty interval.')+figure(5,CAPTIONS[5])+'''
<div class="two-col"><article id="perception-policy"><h3>Perception cost is clear; readaptation is inconclusive.</h3><p>The measured-error injector cost Frozen F '''+pp(v['frozen_cost']['delta_pp'])+''' pp capture. The readaptation estimate was '''+pp(v['readaptation']['mean_pp'])+''' pp and its seed-level interval spans zero. Real-footage error measurement remains a separate offline lineage.</p></article><article id="safety"><h3>Geometry changed crash rate, not the safety contract.</h3><p>Arc-clearance versus riskcap produced a seed-mean crash difference of '''+pp(v['safety_seed']['mean'],4)+''' pp, seed-t 95% CI ['''+', '.join(pp(v['safety_seed'][k]) for k in ['lo','hi'])+''']. The historical pooled estimate is '''+pp(v['safety_pooled']['delta'],4)+''' pp. A lower configured crash rate is not a safety guarantee.</p></article><article id="observation"><h3>Rendering sensitivity belongs to Policy R.</h3><p>The '''+pp(v['observation']['mean'],3)+''' pp capture loss is MATERIAL_LOSS. Policy R is a different checkpoint from F; renderer-defect causality is NOT_TESTED.</p></article><article id="related"><h3>External positioning has no matched leaderboard.</h3><p>Class A matched external comparisons: 0. Published systems study different task and sensor contracts. <a href="https://github.com/joshualikaist/MOTAR/blob/main/docs/relation_to_published_systems_2026-09-16.md">Read the comparison boundaries ↗</a></p></article></div></section>
<section class="chapter boia-chapter" id="boia" data-boia-record="data/boia-progress.json">'''+head('05 / Current research','When should tracking yield to reacquisition?','BOIA investigates this intervention-timing question after a target has already been acquired. First acquisition is outside this extension. The RGB-D + LiDAR contract, Frozen F checkpoint and 898-D actor remain unchanged.')+figure(6,CAPTIONS[6])+'''
<p class="margin-note"><strong>Current operation: shadow-only.</strong> Frozen F owns the applied commands. BOIA utility remains ABSTAIN; reference triggers may log a proposal. REACQUIRE PROPOSAL — PLANNED; no online command.</p>'''+milestones()+'''
<div class="minor-note" style="margin-top:24px">'''+limitations()+'''</div><p class="minor-note" style="margin-top:18px">Development snapshot: local research source <code>'''+v['research_snapshot'][:7]+'''</code>, 22 October 2026. <a href="evidence.html#boia">Review findings and source map ↗</a></p></section>
<section class="chapter" id="figures">'''+head('06 / Publication assets','One figure family. Editable source.','Every figure is rebuilt from the source contract or immutable result records. SVG keeps text editable, PDF embeds fonts, and PNG is exported at 300 dpi. The manifest records source and output hashes.')+download_rows()+'''</section>
<section class="chapter" id="reproducibility">'''+head('07 / Reproducibility','Results stay attached to their contracts.','The historical experiment records, preregistrations, checkpoints and original figure packages remain unchanged. This page redraws the presentation; it does not run a new experiment.')+'''
<div class="two-col"><div><h3>What can be inspected</h3><p><a href="evidence.html">Evidence ledger</a>, <a href="'''+ASSETS+'''/manifest.json">figure data and hashes</a>, <a href="../research/figure-redesign-2026-10-22.md">source verification and corrections</a>, and <a href="data/boia-progress.json">the pinned BOIA progress record</a>. Public code continues to point to the separate MOTAR-public release; the local BOIA implementation is not published by this page.</p></div><div><h3>What remains outside the evidence</h3><p>No real-flight validation, sim-to-real transfer, calibrated identity, BOIA capture benefit or closed-loop safety is established. Minimum-distance results remain WITHHELD_SEMANTIC_MISMATCH. Raw replication evidence is retained outside normal Git history and is not publicly downloadable.</p></div></div></section>
'''+demo+'''</main>'''
    return shell('MOTAR · Tracking, perception and active reacquisition',body,'publication',scripts)


def download_rows():
    rows='<div class="download-list">'
    for i,f in enumerate(MANIFEST['figures'],1):
        rows+='<div class="download-row"><span class="index">'+str(i).zfill(2)+'</span><div><h3><a href="figures.html#figure-'+str(i)+'">'+f['title']+'</a></h3><p>Publication width '+str(f['width_in'])+' in · vector source + 300 dpi raster</p></div>'+files(f['id'])+'</div>'
    return rows+'</div><p style="margin-top:20px"><a class="all-download" href="'+ASSETS+'/motar-publication-figures.zip">Download all six figures + manifest ↓</a></p>'


def build_gallery():
    body='''<main id="main" class="page gallery"><header class="evidence-hero"><p class="eyebrow">MOTAR / Publication figures</p><h1>Figures, rebuilt from source.</h1><p>Six scientific figures for inspection, editing and paper preparation. Each caption states its scope; the underlying records remain unchanged.</p><p style="margin-top:20px"><a class="all-download" href="'''+ASSETS+'''/motar-publication-figures.zip">Download SVG + PDF + PNG bundle ↓</a></p></header><p class="gallery-intro">Select a figure to open the editable SVG. On a narrow screen, scroll inside the figure panel. <a href="../research/figure-redesign-2026-10-22.md">Source verification and correction notes</a> · <a href="'''+ASSETS+'''/manifest.json">Data and SHA-256 manifest</a></p>'''
    body+=''.join(figure(i,CAPTIONS[i]) for i in range(1,7))+'</main>'
    return shell('MOTAR · Publication figure atlas',body,'gallery-page')


def build_evidence():
    v=MANIFEST['values'];q=PROGRESS['q02a'];sf=v['safety_seed'];sp=v['safety_pooled']
    rows=[
        ['Perception → policy','Policy F',pp(v['frozen_cost']['delta_pp'])+' pp; 95% CI ['+', '.join(pp(x) for x in v['frozen_cost']['ci95'])+']','Episode-level interval from two evaluation seeds; one error injector.','perception'],
        ['Readaptation','Policy F',pp(v['readaptation']['mean_pp'])+' pp; 95% CI ['+', '.join(pp(x) for x in v['readaptation']['ci95'])+']','Three training seeds; INCONCLUSIVE.','perception'],
        ['Safety geometry · seed inference','Policy F',pp(sf['mean'],6)+' pp; t 95% CI ['+', '.join(pp(sf[x]) for x in ['lo','hi'])+']','Three independent evaluation seeds. This is the figure estimand.','safety'],
        ['Safety geometry · pooled historical','Policy F',pp(sp['delta'],6)+' pp; pooled 95% CI ['+', '.join(pp(sp[x]) for x in ['lo','hi'])+']','Fifteen seed × density cells; not fifteen independent replicates.','safety'],
        ['Observation sensitivity','Policy R',pp(v['observation']['mean'],3)+' pp; t 95% CI ['+', '.join(pp(x,3) for x in v['observation']['ci95'])+']','Three paired evaluation seeds; MATERIAL_LOSS; causality NOT_TESTED.','observation']]
    numerical='<div class="table-scroll" role="region" aria-label="Historical result ledger" tabindex="0"><table><thead><tr><th>Experiment</th><th>Checkpoint</th><th>Estimate and interval</th><th>Replication unit / boundary</th></tr></thead><tbody>'
    for name,policy,val,limit,src in rows:
        numerical+='<tr><td><a href="https://github.com/joshualikaist/MOTAR/blob/main/'+MANIFEST['sources'][src]['path']+'">'+name+'</a></td><td>'+policy+'</td><td>'+val+'</td><td>'+limit+'</td></tr>'
    numerical+='</tbody></table></div>'
    source_rows=''.join('<tr><td>'+html.escape(k)+'</td><td><code>'+html.escape(s['path'])+'</code></td><td><code>'+s['sha256'][:16]+'…</code></td></tr>' for k,s in MANIFEST['sources'].items())
    body='''<main id="main" class="page"><header class="evidence-hero"><p class="eyebrow">MOTAR / Evidence ledger</p><h1>Claims, sources<br>and their limits.</h1><p>The historical measurements and the BOIA development snapshot answer different questions. This ledger retains their units of replication, source records and unresolved failures.</p></header><div class="evidence-grid"><nav class="toc" aria-label="Evidence sections"><a href="#system">System contract</a><a href="#replication-provenance">Target-motion replication</a><a href="#evidence">Historical studies</a><a href="#boia">BOIA / current gate</a><a href="#sources">Figure provenance</a><a href="#limitations">Limitations</a></nav><div class="evidence-content">
<section id="system"><h2>System and information contract</h2><p id="introduction">MOTAR evaluates frozen UAV tracking and close-approach policies in simulation. Its official sensor contract is RGB-D Camera + LiDAR. LiDAR supports obstacles and target-association assistance; the simulator pipeline is not a qualified live real-image perception stack.</p>
<p id="method">The flat actor observation has 898 values. The network embeds it as 17 tokens, including CLS. The actor produces four components, but the task replaces its z component with PI altitude hold. Policy F uses riskcap; arc-clearance is an alternate experimental mode, not a second serial filter or a planner.</p>
<p id="perception">The tracker contains a six-dimensional position/velocity mean and 6 × 6 covariance. Five sampled history slots nominally span two seconds, while track expiry is five seconds. Simulator own state is not evidence of a calibrated hardware estimator.</p>
<p>Scene GT generates synthetic sensor observations and supplies evaluation, rewards, termination and the privileged critic. Target pose, semantic mask and visibility are not direct actor inputs. Offline real-image error estimation remains a separate lineage. <a href="index.html#representation">Observation structure</a> · <a href="figures.html#figure-1">Corrected system figure</a>.</p>
<p class="source-path">Architecture checked at historical measurement source <code>'''+MANIFEST['measurement_source']+'''</code>. Checkpoint training-source provenance remains UNKNOWN; a measurement source is not a training source.</p></section>
<section id="replication-provenance"><span id="target-motion"></span><h2>Target-motion replication</h2><p>Independent seven-seed, bias-corrected replication: seeds 4104–4110, four arms × four densities, 112 cells, '''+format(v['episodes'],',')+''' primary episodes. H is training-lineage motion; E0 static; E1 bounded constant velocity; E2 bounded obstacle-aware local steering. No PPO training was performed.</p>
<div class="table-scroll"><table><thead><tr><th>Paired contrast</th><th>Capture difference</th><th>BCa 95% interval</th><th>Holm p</th></tr></thead><tbody>'''+''.join('<tr><td>'+ARMLABEL[r['arm']]+' − H</td><td>'+pp(100*r['mean_diff'])+' pp</td><td>['+', '.join(pp(100*x) for x in r['bca95'])+'] pp</td><td>'+str(r['holm_adjusted_p'])+'</td></tr>' for r in v['motion_primary'])+'''</tbody></table></div>
<h3>Target motion and browser boundaries</h3><p>No target law reacts to the pursuer; obstacle-aware is not adversarial. These are separate historical target-motion lineages. <a href="https://github.com/joshualikaist/MOTAR/blob/main/docs/target_motion_algorithm_2026-09-17.md">Canonical target-motion definitions</a>. Browser visualization only: the explanatory browser route is distinct from these policy evaluations.</p><p>Inference unit: evaluation seed. Densities are equally weighted. All three contrasts replicated, but the Holm-adjusted p-values are at the seven-seed exact-test resolution boundary. Density-specific E2 differences are exploratory and mixed at high density.</p>
<p>Original campaign (18 September, n=3, legacy pooled stopping window); bias discovery and canary (instrumentation validation); independent replication (25–26 September, n=7, corrected per-environment quota). The stages remain separate and are never pooled. Raw replication evidence is retained outside normal Git history and is not publicly downloadable.</p>
<p>Measurement commit: <code>'''+MANIFEST['measurement_source']+'''</code>; raw freeze: <code>2f723ed</code>. Archive SHA-256: <code>0536897b418cc75dc2a8cadb0096f15fa83293fa4aabb509add53862b37eeb1b</code>.</p>
<ul class="source-list"><li><a href="https://github.com/joshualikaist/MOTAR/blob/main/results/target_motion_replication_7seed/analysis.json">Canonical analysis</a></li><li><a href="https://github.com/joshualikaist/MOTAR/blob/main/results/target_motion_replication_7seed/RAW_MANIFEST.json">Raw manifest</a> · <a href="https://github.com/joshualikaist/MOTAR/blob/main/results/target_motion_replication_7seed/integrity_report.json">Integrity</a> · <a href="https://github.com/joshualikaist/MOTAR/blob/main/results/target_motion_replication_7seed/primary_selection_manifest.json">Primary selection</a></li></ul>
<p>Minimum distance remains WITHHELD_SEMANTIC_MISMATCH: capture tests swept closest approach; recorded distance samples step ends. The new figures do not display that metric.</p></section>
<section id="evidence"><span id="experiments"></span><h2>Historical measurements</h2><p>pp = percentage points. Each contrast retains its original interval and replication unit; they are not combined into an overall algorithm-performance score.</p>'''+numerical+'''
<p>The safety seed mean and pooled estimate are different estimands. The redesigned figure centers the seed-t interval on the seed mean. Frozen perception-cost evaluations repeated across training campaigns are a determinism check, not extra independent replications.</p>
<h3 id="environment">Real-footage measurement boundary</h3><p>The apparent-size range proxy records '''+format(v['e3s']['median_pct'],'.1f')+'''% as the median of '''+str(v['e3s']['blocks'])+''' block medians from one flight: '''+format(v['e3s']['frames'],',')+''' frames in '''+str(v['e3s']['block_seconds'])+'''-second blocks. Frames are not independent replicates. The real-image detector and temporal-selector lineages are not silently substituted into the simulation loop.</p>
<p>Class A matched external comparisons: 0. <a href="https://github.com/joshualikaist/MOTAR/blob/main/docs/EVIDENCE.md">Full historical evidence summary</a> · <a href="https://github.com/joshualikaist/MOTAR/blob/main/docs/relation_to_published_systems_2026-09-16.md">Relation to published systems</a> · <a href="renderer-contract-v1.html">Separate renderer characterization</a>.</p></section>
<section id="boia" data-boia-record="data/boia-progress.json"><h2>BOIA development / HOLD</h2><p>The local research snapshot is <code>'''+v['research_snapshot']+'''</code>, dated 22 October 2026. The current branch is shadow-only. First acquisition is outside its scope; Frozen F remains in control. This page does not publish local research commits or their implementation.</p>'''+milestones()+'''
<h3>Q02-A review boundary</h3><p>'''+str(q['focused_cpu']['run'])+'/'+str(q['focused_cpu']['passed'])+''' focused CPU tests PASS. Earlier PUBLIC_CPU: '''+format(q['research_public_cpu']['run'],',')+''' tests, '''+str(q['research_public_cpu']['failures'])+''' failures, '''+str(q['research_public_cpu']['errors'])+''' existing errors; aggregate FAIL. Final review: PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS; residual P2 H1/H2. These are contract checks, not an occlusion-accuracy or same-target-identity validation.</p>
<h3>Q02-B/Q03 stage-gate review</h3><p>112/112 tests OK across six focused suites: Q02-A 49, Q02-B 29, Map: 12 CPU tests, planner: 15 CPU tests, benchmark metrics 2, non-interference 5. Independent review found 0 P0, 2 P1 and 9 P2; stage gate FAIL/HOLD. Both P1 probes were reproduced and remain unfixed.</p><ul><li>NOT_TESTABLE decisions incorrectly advance loss clocks as misses.</li><li>Candidate L2 screens omit current-velocity stopping/cancel displacement.</li></ul>
<p>Latest research PUBLIC_CPU: 3,177 run, 0 failures, 10 skips, 4 errors; aggregate FAIL. The focused-suite results do not override the independent review. L3/L4 and a physical free-volume certificate remain unvalidated; there is no action executor.</p>'''+limitations()+'''
<p><a href="../research/boia-progress.md">Research summary and source map</a> · <a href="data/boia-progress.json">Source paths, commits and content hashes</a>. Public-release CPU checks and research PUBLIC_CPU are separate profiles.</p></section>
<section id="sources"><h2>Figure provenance</h2><p>Six new figures, each exported as editable SVG, embedded-font PDF and 300 dpi PNG. The density panel is a new exploratory display of immutable primary counts; it introduces no new confirmatory test or confidence interval.</p><div class="table-scroll"><table><thead><tr><th>Record</th><th>Source path</th><th>SHA-256 prefix</th></tr></thead><tbody>'''+source_rows+'''</tbody></table></div><p><a href="'''+ASSETS+'''/manifest.json">Full manifest and plotted values</a> · <a href="../research/figure-redesign-2026-10-22.md">File/function verification and corrected errors</a> · <a href="figures.html">Figure atlas</a>.</p></section>
<section id="limitations"><span id="reproducibility"></span><h2>Limits and availability</h2><ul><li>Simulation only; no real-flight validation, hardware safety, sim-to-real or state-of-the-art claim.</li><li>Policy F and R, historical experiments and BOIA development are distinct lineages.</li><li>No causal attribution for D8b, no additive interpretation across experiments, no recovered persistent identity.</li><li>External raw backup remains absent; raw and checkpoints were not changed by this documentation task.</li><li>The <a href="index.html#demo">BROWSER GT PREVIEW</a> is NOT PPO, NOT PHYSX and not research evidence.</li></ul><p>Public code points to the separate <a href="https://github.com/joshualikaist/MOTAR-public">MOTAR-public</a> repository. Local BOIA research implementation is not included. <a href="https://github.com/joshualikaist/MOTAR/blob/main/docs/REPRODUCIBILITY.md">Reproducibility instructions</a>.</p></section>
</div></div></main>'''
    body=body.replace('<h3 id="environment">',registries()+'<h3 id="environment">')
    return shell('MOTAR · Evidence and provenance',body,'evidence-page','<script src="status_manifest.js?v=20260924" defer></script>')


def main():
    global MANIFEST,PROGRESS,ARMLABEL
    MANIFEST=read('docs/assets/paper/redesign-2026-10-22/manifest.json')
    PROGRESS=read('docs/status/data/boia-progress.json')
    ARMLABEL={'E0_static':'E0','E1_cv':'E1','E2_obstacle_aware':'E2'}
    for name,builder in [('index.html',build_index),('evidence.html',build_evidence),('figures.html',build_gallery)]:
        (ROOT/'docs/status'/name).write_text(builder())
    print('PASS: index.html, evidence.html, figures.html generated from pinned documentation records')


if __name__=='__main__':main()
