#!/usr/bin/env python3
"""Read-only, stdlib existing-data audit. Writes metrics and original-file bindings externally."""
import ast,collections,gzip,hashlib,json,os,struct,subprocess,time
from pathlib import Path
W=Path('/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly')
T=Path('/home/fair/motar_tmp'); O=T/'sitl_next_20261110/data_audit_r1'
bindings={}
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def bind(p):
 p=Path(p)
 if str(p) not in bindings:bindings[str(p)]={'bytes':p.stat().st_size,'sha256':sha(p)}
 return bindings[str(p)]['sha256']
def jread(p):bind(p);return json.loads(Path(p).read_text())
def rows(p):
 bind(p)
 with Path(p).open() as f:return [json.loads(l) for l in f if l.strip()]
def npy(p):
 bind(p)
 with p.open('rb') as f:
  magic=f.read(8);n=struct.unpack('<H' if magic[6]==1 else '<I',f.read(2 if magic[6]==1 else 4))[0];hdr=ast.literal_eval(f.read(n).decode());body=f.read()
 assert hdr['descr']=='<f8' and not hdr['fortran_order'];n,m=hdr['shape'];assert len(body)==n*m*8
 return hdr,list(struct.iter_unpack('<'+'d'*m,body))
def manifest(p,root):
 d=jread(p);miss=[];bad=[];size=0
 for k,v in d['files'].items():
  f=root/k
  if not f.is_file():miss.append(str(f));continue
  size+=f.stat().st_size
  if bind(f)!=v['sha256'] or f.stat().st_size!=v.get('bytes',f.stat().st_size):bad.append(str(f))
 return {'manifest':str(p),'root':str(root),'expected_files':len(d['files']),'checked_present_bytes':size,'missing':miss,'mismatched':bad}
def ulog_inventory(p):
 """Parse ULog subscription/format/data headers without pyulog or reconstruction."""
 op=gzip.open if p.suffix=='.gz' else open
 bind(p);forms={};subs={};cnt=collections.Counter();ts={};err=[];dropouts=[]
 with op(p,'rb') as f:
  header=f.read(16)
  assert header[:7]==b'ULog\x01\x12\x35'
  while True:
   h=f.read(3)
   if not h:break
   if len(h)!=3:err.append('truncated header');break
   n,typ=struct.unpack('<HB',h);b=f.read(n)
   if len(b)!=n:err.append('truncated payload');break
   if typ==ord('O'):
    dropouts.append(struct.unpack('<H',b[:2])[0])
   elif typ==ord('F'):
    s=b.decode(errors='replace');name,fmt=s.split(':',1);forms[name]=fmt
   elif typ==ord('A'):
    multi,mid=struct.unpack('<BH',b[:3]);subs[mid]=b[3:].decode(errors='replace')
   elif typ==ord('D'):
    mid=struct.unpack('<H',b[:2])[0];name=subs.get(mid,'unknown');cnt[name]+=1
    if len(b)>=10:
     stamp=struct.unpack('<Q',b[2:10])[0];a=ts.setdefault(name,[stamp,stamp]);a[1]=stamp
 out={}
 for name,n in cnt.items():
  a=ts.get(name);span=(a[1]-a[0])/1e6 if a else None
  out[name]={'messages':n,'span_s':span,'approx_message_rate_hz':(n-1)/span if span and span>0 else None,'format':forms.get(name)}
 return {'path':str(p),'scope':'ONE actual representative ULog per family; topic availability/rates, not reconstruction or all-episode guarantee','parse_errors':err,'dropout_events':len(dropouts),'dropout_total_ms':sum(dropouts),'dropout_max_ms':max(dropouts,default=0),'topics':out}
metrics={'protocol':str(O/'PROTOCOL.md'),'generated_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'manifests':{},'cam':{},'sitl':{}}
for name,ext,folder,mname in [('K1E','camk1e_20261108','camk1e','SHA256_MANIFEST.json'),('K2E','camk2e_20261108','camk2e','SHA256_MANIFEST.json'),('K2E3','camk2v3e_20261110','camk2e','SHA256_MANIFEST_v3e.json'),('S4F1','s4f1_20261108',None,'SHA256_MANIFEST.json')]:
 p=W/'docs/evidence'/('camonly_2026-11-04/'+folder if folder else 'sitl_2026-11-05/s4f1')/mname
 metrics['manifests'][name]=manifest(p,T/ext)
 print('manifest',name,flush=True)
for name,ext in [('K1E','camk1e_20261108'),('K2E','camk2e_20261108'),('K2E3','camk2v3e_20261110')]:
 root=T/ext/'raw/eval_01';cells=[];arms={};allkeys=set();kkeys=set()
 for p in sorted(root.rglob('episodes.jsonl')):
  rr=rows(p);allkeys.update(*(r.keys() for r in rr));q=[r for r in rr if r['in_quota']];summary=jread(p.parent/'episode_ledger_summary.json');ids=collections.Counter(r['episode_id'] for r in rr);env=collections.Counter(r['env_index'] for r in q);ct=collections.Counter(r['outcome'] for r in q);kr=rows(p.parent/'kroc2027_episodes.jsonl');kkeys.update(*(r.keys() for r in kr));kmap={(r['env_index'],r['env_episode_index']):r for r in kr};paired=sum((r['env_index'],r['env_episode_index']) in kmap for r in q)
  assert len(q)==2048 and len(env)==128 and set(env.values())=={16};assert not any(n>1 for n in ids.values());assert ct=={k:v for k,v in summary['quota']['counts'].items() if v};assert len(rr)==summary['rows_written'];assert paired==len(q)
  c={'path':str(p),'arm':p.parent.parent.name,'bars':q[0]['bars'],'seed':q[0]['seed'],'all_rows':len(rr),'quota_rows':len(q),'outside_quota_rows':len(rr)-len(q),'duplicate_episode_ids':sum(n-1 for n in ids.values()),'outcomes':dict(ct),'kroc_rows':len(kr),'quota_kroc_joined':paired,'unique_envs':len(env),'quota_per_env':16,'summary_status':summary['status']};cells.append(c)
  a=arms.setdefault(c['arm'],{'cells':0,'quota_rows':0,'outcomes':collections.Counter()});a['cells']+=1;a['quota_rows']+=len(q);a['outcomes'].update(ct)
 for a in arms.values():a['outcomes']=dict(a['outcomes']);a['rates_pct']={k:100*v/a['quota_rows'] for k,v in a['outcomes'].items()}
 metrics['cam'][name]={'cells':cells,'arms':arms,'ledger_schema_keys':sorted(allkeys),'kroc_schema_keys':sorted(kkeys),'status':'PASS scoped quota count/partition/uniqueness/summary/join checks','all_rows':sum(c['all_rows'] for c in cells),'quota_rows':sum(c['quota_rows'] for c in cells),'primary_rule':'first16 completed episodes per each128 envs; exclude in_quota=false rows without deleting them'}
 print('cam',name,flush=True)
ledgers={}
for name,ext in [('S3b','s3b_formal_20261106'),('S3c','s3c_20261106'),('S4','s4_20261106'),('S4F1','s4f1_20261108')]:
 root=T/ext/'attempt01';rr=rows(root/'episodes.jsonl');ledgers[name]=rr;attempt=jread(root/'attempt.json');final=jread(root/'final.json');group={};inv=[];nsteps=0;nsens=0;shapes=collections.Counter();sensor_shapes=collections.Counter();ulog_files=[];missing=[];stepchecks=[]
 for r in rr:
  a=group.setdefault(r.get('block',r['arm'] if 'arm'in r else '?'),{'scheduled':0,'valid':0,'all_outcomes':collections.Counter(),'valid_outcomes':collections.Counter(),'invalid_reasons':collections.Counter()});a['scheduled']+=1;a['valid']+=bool(r['valid']);a['all_outcomes'][r['outcome']]+=1
  if r['valid']:a['valid_outcomes'][r['outcome']]+=1
  else:a['invalid_reasons'][str(r['invalid_reason'])]+=1;inv.append({k:r.get(k) for k in ['index','env','arm','block','outcome','invalid_reason','late_max_s']})
  ep=root/('ep%03d'%r['index']);p=ep/'steps.npy'
  if p.exists():
   h,data=npy(p);nsteps+=len(data);shapes[str(h['shape'][1])]+=1;stepchecks.append(len(data)==r['steps'])
  else:missing.append(str(p))
  p=ep/'sensor_steps.npy'
  if p.exists():h,sdata=npy(p);nsens+=len(sdata);sensor_shapes[str(h['shape'][1])]+=1;stepchecks.append(len(sdata)==r['steps'])
  ulog_files.extend(sorted(ep.rglob('*.ulg'))+sorted(ep.rglob('*.ulg.gz')))
  for small in ['session.json','launches.jsonl','logs/px4.log','logs/gzserver.log']:
   if (ep/small).exists():bind(ep/small)
 assert len(rr)==final['episodes_recorded'];assert len({r['index'] for r in rr})==len(rr);assert all(stepchecks)
 for a in group.values():
  for k in ['all_outcomes','valid_outcomes','invalid_reasons']:a[k]=dict(a[k])
 ug=ulog_inventory(ulog_files[0]) if ulog_files else None
 required=['vehicle_attitude_groundtruth','vehicle_local_position_groundtruth','vehicle_local_position','vehicle_attitude','vehicle_rates_setpoint','vehicle_command','vehicle_command_ack','vehicle_status','control_allocator_status','actuator_motors']
 ulogs=[]
 for up in ulog_files:
  uv=ug if up==ulog_files[0] else ulog_inventory(up)
  ulogs.append({'path':str(up),'topic_count':len(uv['topics']),'missing_required_topics':[k for k in required if k not in uv['topics']],'parse_errors':uv['parse_errors'],'dropout_events':uv['dropout_events'],'dropout_total_ms':uv['dropout_total_ms'],'dropout_max_ms':uv['dropout_max_ms']})
 metrics['sitl'][name]={'root':str(root),'ledger_rows':len(rr),'schedule_rows':len(attempt['schedule']),'final':final,'groups':group,'invalid_rows':inv,'duplicate_indices':len(rr)-len({r['index'] for r in rr}),'step_rows':nsteps,'sensor_rows':nsens,'step_width_counts':dict(shapes),'sensor_width_counts':dict(sensor_shapes),'missing_step_files':missing,'step_count_checks':len(stepchecks),'step_count_check_failures':sum(not x for x in stepchecks),'ledger_schema_keys':sorted(set().union(*(r.keys() for r in rr))),'ulog_files':len(ulog_files),'ulog_files_paths':[str(p) for p in ulog_files],'representative_ulog':ug,'all_ulog_topic_dropout_checks':ulogs,'late_max_s':max(r['late_max_s'] for r in rr),'rtf_range':[min(r['real_time_factor'] for r in rr),max(r['real_time_factor'] for r in rr)]}
 print('sitl',name,flush=True)
# Check compact execution-record original-file pins, with no claim about mutable stack files.
metrics['compact_pins']={}
for family,path in [('S3b','s3b_formal'),('S3cS4','s3c_s4')]:
 d=jread(W/'docs/evidence/sitl_2026-11-05'/path/'execution_record.json');arr=[d] if family=='S3b' else [d['s3c'],d['s4']]
 checks=[]
 for a in arr:
  root=Path(a['raw']['attempt_dir'])
  for k,v in a['raw'].items():
   if k=='attempt_dir':continue
   checks.append({'path':str(root/k),'matches':bind(root/k)==v})
 metrics['compact_pins'][family]=checks
s4={r['env']:r for r in ledgers['S4'] if r['arm']=='ground' and r['valid']};f={r['env']:r for r in ledgers['S4F1'] if r['arm']=='lit_rendered' and r['valid']};common=sorted(s4.keys()&f.keys());trans=collections.Counter((s4[e]['outcome'],f[e]['outcome']) for e in common);n=len(common);m1=collections.Counter(s4[e]['outcome'] for e in common);m2=collections.Counter(f[e]['outcome'] for e in common);agree=sum(s4[e]['outcome']==f[e]['outcome'] for e in common);pe=sum(m1[k]*m2[k] for k in set(m1)|set(m2))/n**2
metrics['repeatability']={'S4_ground_vs_S4F1_lit_rendered':{'n':n,'same_outcome':agree,'different_outcome':n-agree,'transitions':{' -> '.join(k):v for k,v in trans.items()},'S4_outcomes':dict(m1),'S4F1_outcomes':dict(m2),'expected_agreement_n':n*pe,'kappa':(agree/n-pe)/(1-pe)},'S3b_A_vs_C':{'n':len([r for r in ledgers['S3b'] if r['block']=='C']),'same_outcome':sum(r['outcome']==next(a['outcome'] for a in ledgers['S3b'] if a['block']=='A' and a['env']==r['env']) for r in ledgers['S3b'] if r['block']=='C')}}
# bind prior E3 independent audit and AGQ reconstruction records
for p in [W/'docs/evidence/camonly_2026-11-04/camk2e/review_summary_v3e_r1.json',W/'docs/evidence/sitl_2026-11-05/dev08/agq/diagnosis.json',W/'docs/evidence/sitl_2026-11-05/dev08/agq/geometry_v1/execution_record.json']:jread(p)
(O/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n');(O/'ORIGINAL_MANIFEST.json').write_text(json.dumps({'files':bindings,'count':len(bindings)},indent=2)+'\n');print('DONE bindings',len(bindings),flush=True)
