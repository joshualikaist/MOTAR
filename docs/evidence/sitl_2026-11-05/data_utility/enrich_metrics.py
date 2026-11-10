#!/usr/bin/env python3
"""Add bounded supplementary original-file checks and utility contract to audit_data.py output."""
from pathlib import Path
import collections,json,hashlib,gzip,struct,ast
O=Path(__file__).parent;T=Path('/home/fair/motar_tmp');W=Path('/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly')
m=json.loads((O/'metrics.json').read_text());manifest=json.loads((O/'ORIGINAL_MANIFEST.json').read_text());bindings=manifest['files']
def bind(p):
 p=Path(p);h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 bindings[str(p)]={'bytes':p.stat().st_size,'sha256':h.hexdigest()};return h.hexdigest()
def rd(p):bind(p);return json.loads(p.read_text())
def rs(p):bind(p);return [json.loads(l) for l in p.open() if l.strip()]
# Historical raw ledger byte duplicates: identity controls are deliberate and excluded from primary N.
m['duplicate_ledger_bytes']={}
for family,ext in [('K1E','camk1e_20261108'),('K2E','camk2e_20261108'),('K2E3','camk2v3e_20261110')]:
 groups=collections.defaultdict(list)
 for p in sorted((T/ext/'raw').rglob('episodes.jsonl')):groups[bindings[str(p)]['sha256']].append(str(p))
 m['duplicate_ledger_bytes'][family]={'all_raw_ledger_files':sum(map(len,groups.values())),'groups':[g for g in groups.values() if len(g)>1],'primary_eval_duplicate_groups':[g for g in groups.values() if sum('/eval_01/' in p for p in g)>1]}
# Confirm two structural truncations are historical decompressed bytes and after flat terminal sample.
tails=[];rr=rs(T/'s4f1_20261108/attempt01/episodes.jsonl')
for ix in [4,139]:
 r=rr[ix];ep=T/'s4f1_20261108/attempt01'/('ep%03d'%ix);p=next(ep.rglob('*.ulg.gz'));info=r['ulog']['files'][0];h=hashlib.sha256();data=gzip.decompress(p.read_bytes());h.update(data);subs={};off=16;lastcomplete={};tail={}
 while off<len(data):
  n,ty=struct.unpack('<HB',data[off:off+3]);b=data[off+3:off+3+n]
  if ty==65:subs[struct.unpack('<H',b[1:3])[0]]=b[3:].decode()
  if ty==68 and len(b)>=10:
   mid,ts=struct.unpack('<HQ',b[:10]);topic=subs.get(mid);stamp=ts/1e6
   if len(b)==n:lastcomplete[topic]=stamp
   else:tail={'topic':topic,'timestamp_s':stamp,'expected_payload_bytes':n,'actual_payload_bytes':len(b)}
  if len(b)!=n:break
  off+=3+n
 with (ep/'steps.npy').open('rb') as f:
  f.read(8);z=struct.unpack('<H',f.read(2))[0];hdr=ast.literal_eval(f.read(z).decode());values=list(struct.iter_unpack('<'+'d'*27,f.read()))
 tails.append({'episode':ix,'env':r['env'],'arm':r['arm'],'outcome':r['outcome'],'path':str(p),'decompressed_sha256_matches_recorded_original':h.hexdigest()==info['sha256'],'decompressed_bytes':len(data),'last_step_px4_s':values[-1][1],'tail':tail,'last_complete_truth_position_s':lastcomplete['vehicle_local_position_groundtruth'],'limit':'tail incomplete after terminal; outcome retained; no complete cleanup-tail or exact losslessness claim'})
m['ulog_tail_findings']=tails
# Original dev08 ledger and four actual prior diagnosis cases: no synthetic replay.
dp=T/'sitl_s3b_20261105/dev08_prefix_attempt01/ledger.jsonl';dr=rs(dp);groups={}
for r in dr:
 k=r['phase']+'/'+r['arm'];g=groups.setdefault(k,{'rows':0,'outcomes':collections.Counter(),'operational_status':collections.Counter()});g['rows']+=1;g['outcomes'][r['outcome']]+=1;g['operational_status'][r['operational_status']]+=1
for g in groups.values():g['outcomes']=dict(g['outcomes']);g['operational_status']=dict(g['operational_status'])
diag=rd(T/'agq_diagnosis_20261106/agent_analysis_r1/analysis_v3/diagnosis.json');checks=[]
for c in diag['cases']:
 for label in ['ulog','checkpoints','world']:
  x=c[label];checks.append({'kind':label,'path':x['path'],'matches_prior_binding':bind(x['path'])==x['sha256']})
m['dev08_AGQ']={'ledger_path':str(dp),'rows':len(dr),'groups':groups,'diagnostic_original_checks':checks,'existing_diagnosis_scope':'four historical205/env2,env5 ULogs; orientation/tracking/footprint proxies recoverable; exact contact and transport time NOT_RECORDED there'}
geom=T/'agq_geometry_diag_20261106/attempt01';gr=rs(geom/'ledger.jsonl');members=[]
for ep in sorted(geom.glob('0*')):
 if not ep.is_dir():continue
 g={ 'member':ep.name,'files':{}}
 for name in ['contacts.jsonl','mavlink_sends.jsonl','mavlink_arrivals.jsonl','recording.json']:
  p=ep/'recording'/name
  if name=='recording.json':v=rd(p);g['recording_counts']=v.get('counts');g['recording_clock']=v.get('clock');continue
  r=rs(p);g['files'][name]={'path':str(p),'rows':len(r),'schema_example':r[0] if r else None}
 members.append(g)
m['dev08_AGQ']['geometry_v1']={'ledger_rows':len(gr),'members':members,'scope':'actual contact/MAVLink available for four later development members only; not a substitute for olddev08/S4/F1 missing streams','historical_label':'INCONCLUSIVE_NOT_REPRODUCED bothpairs'}
m['utility_matrix']=[
 {'data':'CAM-K1-E/K2-E/E3','fixed_grid_outcomes':'USEFUL_WITH_REGISTERED_SCOPE','inference_unit':'12density×evaluation-seed blocks per checkpoint pair (nominal K2:6); only2trainingseeds; UNPAIRED_STREAM rows not matched scene repetitions','failure_mechanism':'INSUFFICIENT_CAUSAL_EVIDENCE: no aligned per-quota hitbar/token/FOV/clearance/pose trace; aggregate/episode summaries only','real_sensor_transfer':'NOT_TESTED: simulated model, no realOAK-D bench inputs'},
 {'data':'S3b formal/S3c','fixed_grid_outcomes':'USEFUL_WITH_LIMITS: fixed eligible G8 scenes; S3a NOT_EQUIVALENT, rotor contacts not identical to logical crash','inference_unit':'oneboot per fixedscene/arm except10S3bC repeatboots; no independentpopulation/training replication','failure_mechanism':'PARTLY_RECONSTRUCTIBLE via originalULogs+steps+world; actualcontact/packetphase absent; no causal attribution','real_sensor_transfer':'NOT_TESTED'},
 {'data':'S4/S4F1','fixed_grid_outcomes':'USEFUL_WITH_LIMITS: valid outcomes and exclusions, fixedbalanced condition rates; historicalHolm tests unchanged','inference_unit':'40fixedscenes perF1cell, oneboot/cell; weakscene reproducibility limits deterministic pairedcase reasoning/power','failure_mechanism':'PARTLY_RECONSTRUCTIBLE trajectory/attitude/sensorages; rawframes/obs/transport/contact absent; multiplefactors/confounding remain','real_sensor_transfer':'NOT_TESTED'},
 {'data':'dev08/AGQ','fixed_grid_outcomes':'DESCRIPTIVE_ONLY observedprefix (2smokeexcluded+40main); geometry4memberdiagnostic not reproduced historicalfailures','inference_unit':'20fixedscenepairs;4later diagnosticmembers','failure_mechanism':'offlineULog+footprint proxies useful; latergeometryactualcontact positivecontrol exists; intervention INCONCLUSIVE_NOT_REPRODUCED','real_sensor_transfer':'NOT_TESTED'}]
m['coverage_gaps']={
 'CAM_primary_quota':{'recorded':['episodeID','env/repeat','in_quota','completion_step','capture/crash/timeout','minrelative-targetdistance','KROC initialscenehash and episodevisibility/detection/speed counts','FOV/mount/rate/depthmodel settings in receipts/summaries/logs'], 'NOT_RECORDED':['perquota contactbarID/collisionpair','contact-time obstacle token and hitbar visibility/FOV','contact-time perceived/true obstacleclearance','timealigned fullpose/action/observation/sensorframes'],'scope':'KROC swept_min_distance is target distance; target_swept_min_obstacle_clearance is target clearance; neither substitutes pursuer hitbar clearance'},
 'S3b_S3c_S4_F1':{'recorded':['27col steps (k,PX4state time,estimatedxyz/vel/yaw,policyaction,governedvx/vy/yawrate,targetxyz,distance,clearance,computems,visibility,truthxyz/yaw)','S4/F1 16col sensor timestamps/pixel counts/visibleflags/aggregate differences/removedreturns/simnow','ULog estimated and groundtruth quaternion/positions/velocities, rateSP, allocator,motors,command,ACK,status'], 'RECONSTRUCTIBLE':['bodytilt and tracking timeline fromULog native/publication timestamps','geometryoverlap proxy fromworld+ULog orientation/position','sampledsensorage/cadence and flat decision/action trajectory'], 'NOT_RECORDED':['historical actualGazebocontactstream','historicalRGB/depth/LiDARfullarrays and exactactor observation','exactoldTXsend/RXarrival packetclock association','frameID/acquisition-arrival-use lineage/dropcounters/renderseed/phase snapshot'],'limits':'publication timestamp is not physics contactinstant; interpolate only with explicit model/uncertainty; ULog internal command/ACK/status does not recover networkarrival'},
 'real_OAK_D':{'status':'NOT_AVAILABLE_PER_USER','qualification':'NOT_TESTED','required':['realRGB+rawdepth+confidence/invalid mask and aligned timestamps','device+firmware+calibration+intrinsics/extrinsics+settings+rates','controlleddistance/lighting/texture/nearMinZ cases; frame loss and synchronization metadata']}}
m['audit_status']={'checks':'PASS scoped manifests/quota/ledger/step counts; LIMITED_VALIDITY ULog tails/reconstruction and scientific usefulness','independent_review':'PENDING parent fresh review','thresholds_tuned':False,'raw_mutations':False,'no_synthetic_fill':True,'kernel_faults_before':0,'kernel_faults_after':0,'kernel_count_method':'journalctl -k -b0 regex Machine check|segfault|general protection; empty matches counted zero; no hiddentimer'}
(O/'metrics.json').write_text(json.dumps(m,indent=2)+'\n');manifest={'count':len(bindings),'files':bindings};(O/'ORIGINAL_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n');print('enriched',len(bindings),'bindings; primary duplicates',[len(x['primary_eval_duplicate_groups']) for x in m['duplicate_ledger_bytes'].values()])
