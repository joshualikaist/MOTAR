"""Audit cached replay or complete RGB pipeline on validation only."""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from perception_candidates import sha256_file, read_jsonl
from perception_temporal import load_aligned_records, load_aligned_motion_records, TemporalCandidateDataset
from perception_streaming import FrozenDetector, StreamingSelector, PerceptionPipeline
from train_perception_temporal import association_metrics, model_logits


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root', type=Path, required=True)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--mode', choices=['replay','rgb'], required=True)
    p.add_argument('--weights', type=Path)
    p.add_argument('--yolov5', type=Path)
    a = p.parse_args()
    if a.output.exists():
        raise ValueError('refusing existing output')
    d=a.data_root
    m=d/'nps_val_sequence.jsonl'; c=d/'nps_val_candidates.jsonl.gz'
    meta=json.loads(Path(str(m)+'.receipt.json').read_text())
    if meta['split'] != 'val': raise ValueError('validation only')
    aligned, _, candidates_meta=load_aligned_records(m,str(m)+'.receipt.json',c,str(c)+'.receipt.json')
    receipt=json.loads((a.run/'receipt.json').read_text())
    if receipt['validation_candidates_sha256'] != candidates_meta['output_sha256']:
        raise ValueError('candidate/checkpoint mismatch')
    selector=StreamingSelector(a.run/'best.pt',receipt['checkpoint_sha256'])
    mot=d/'nps_val_motion_v2.jsonl.gz'
    motions, motion_meta=load_aligned_motion_records(mot,str(mot)+'.receipt.json',aligned,
                          meta['manifest_sha256'],candidates_meta['output_sha256'])
    if receipt['validation_motion_sha256'] != motion_meta['output_sha256']:
        raise ValueError('motion/checkpoint mismatch')
    offline=TemporalCandidateDataset(aligned,16,.3,motions)
    if a.mode=='rgb':
        if a.weights is None or a.yolov5 is None: raise ValueError('RGB requires detector paths')
        pipeline=PerceptionPipeline(FrozenDetector(a.weights,receipt['detector_weights_sha256'],a.yolov5),selector)
    reference=list(read_jsonl(a.run/'validation_predictions.jsonl.gz'))
    predicted, latencies, rank_mismatches, max_logit_error = [], [], 0, 0.
    candidate_mismatches=0
    for i,row in enumerate(aligned):
        s, candidates=row['source'],row['candidate_record']['candidates']
        began=time.perf_counter()
        if a.mode=='replay':
            out=selector.step(candidates,s['source_sequence_id'],s['capture_timestamp_ns'],
                              s['width_px'],s['height_px'],motions[i])
            batch={k:v.unsqueeze(0).to(selector.device) for k,v in offline[i].items()}
            with torch.no_grad(): expected=model_logits(selector.model,batch)[0].cpu().numpy()
            error=float(np.max(np.abs(np.asarray(out['logits'])-expected)))
            max_logit_error=max(max_logit_error,error)
            if error>1e-5: raise ValueError('replay logits differ')
        else:
            path=str(Path(meta['dataset'])/s['image'])
            frame=cv2.imread(path)
            gray=cv2.imread(path,cv2.IMREAD_GRAYSCALE)
            if frame is None or gray is None: raise ValueError('image decode failed')
            decode_ms=(time.perf_counter()-began)*1000
            out=pipeline.process(frame,s['source_sequence_id'],s['capture_timestamp_ns'],gray)
            out['latency_ms']['decode']=decode_ms
            out['latency_ms']['with_decode']=out['latency_ms']['total']+decode_ms
            latencies.append(out['latency_ms'])
            if out['candidates'] != candidates: candidate_mismatches+=1
            # Metrics must use the candidates actually produced, never cached box coordinates.
            row['candidate_record']={'candidates':out['candidates']}
        rank=out['predicted_rank']
        predicted.append(rank if rank is not None else 5)
        rank_mismatches+=rank!=reference[i]['predicted_rank']
        if (i+1)%500==0:print(a.mode,i+1,flush=True)
    metrics,_=association_metrics(type('Rows',(),{'records':aligned})(),predicted,.3)
    timing={}
    for name in (latencies[0] if latencies else []):
        values=[x[name] for x in latencies]
        timing[name]={'mean':float(np.mean(values)),'p50':float(np.percentile(values,50)),
                      'p95':float(np.percentile(values,95))}
    report={'mode':a.mode,'frames':len(aligned),'metrics':metrics,'rank_mismatches':rank_mismatches,
            'max_replay_logit_error':max_logit_error if a.mode=='replay' else None,
            'candidate_frame_mismatches':candidate_mismatches if a.mode=='rgb' else None,
            'latency_ms':timing,'checkpoint_sha256':receipt['checkpoint_sha256'],
            'manifest_sha256':meta['manifest_sha256'],'test_used':False,
            'gray_contract':'JPEG decoded grayscale for parity with motion v2 cache; live BGR uses cvtColor',
            'latency_scope':'serial batch-one workstation; includes cold first frame; no camera transport',
            'fps_with_decode':1000/timing['with_decode']['mean'] if timing else None}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if rank_mismatches: raise ValueError('offline/online ranks differ')


if __name__=='__main__':main()
