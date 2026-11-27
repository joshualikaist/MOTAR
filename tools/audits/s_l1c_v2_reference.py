#!/usr/bin/env python3
"""Independent NumPy S-L1C v2 candidate oracle. No simulator or production imports.

Association functions receive only a scan and the track's predicted relative centre. Scene
geometry and first-hit labels belong to the separate fixture/evaluation functions below.
"""
from __future__ import annotations
import itertools
import math
from pathlib import Path
import sys
import json
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s_l1c_reference as v1

RADIUS = .20
FAR = 12.
GATE = .35 + 2 * math.sqrt(.03)
CANDIDATES = ('V2-0', 'V2-A', 'V2-B', 'V2-C')


def direction(bearing_deg, elevation_deg):
    b, e = np.radians([bearing_deg, elevation_deg])
    return np.array([math.cos(e)*math.cos(b), math.cos(e)*math.sin(b), math.sin(e)])


def _base(scan, prediction, gate):
    result = dict(v1.associate(scan, prediction, gate_m=gate))
    result.setdefault('row', -1)
    result.setdefault('measured_surface', FAR)
    result.setdefault('centre_range', FAR + RADIUS)
    result['decision'] = 'ACCEPT_V1' if result['valid'] else 'REJECT_V1'
    return result


def _sphere_ranges(center, rays):
    # Unit-ray quadratic, independent from the centre-inversion calculation below.
    projection = rays @ center
    discr = projection**2 - (float(center @ center) - RADIUS**2)
    ranges = projection - np.sqrt(np.maximum(discr, 0))
    hit = (discr >= 0) & (ranges > 0) & (ranges < FAR)
    return np.where(hit, ranges, FAR), hit


def associate(scan, prediction, candidate='V2-A', *, noise_bound_m=.0001, gate_m=GATE):
    """One sensor-only candidate decision. A/B retain v1 acceptances exactly."""
    scan = np.asarray(scan, dtype=np.float64)
    prediction = np.asarray(prediction, dtype=np.float64)
    if (scan.shape != (4, 72) or prediction.shape != (3,) or not np.isfinite(prediction).all()
            or not math.isfinite(noise_bound_m) or noise_bound_m < 0):
        raise ValueError('invalid scan, prediction or noise bound')
    if candidate not in CANDIDATES:
        raise ValueError('unknown candidate')
    base = _base(scan, prediction, gate_m)
    if candidate == 'V2-0' or base['valid']:
        return base
    out = dict(base, valid=False, decision='ABSTAIN_SUPPORT')
    distance = float(np.linalg.norm(prediction))
    if distance < v1.MIN_CONCLUSIVE_M:
        out['decision'] = 'ABSTAIN_RANGE'
        return out
    unit = prediction / distance
    h = out['column']
    column = scan[:, h]
    measured_ok = np.isfinite(column) & (column > 0) & (column < FAR*.995)
    if candidate == 'V2-C':
        elevation = math.atan2(unit[2], math.hypot(unit[0], unit[1]))
        eligible = abs(v1.ELEVATIONS - elevation) < math.asin(min(1., RADIUS/distance))
        if (eligible & measured_ok).any():
            row = int(np.argmin(np.where(eligible & measured_ok, column, np.inf)))
            t = float(column[row])
            if abs(t - (distance-RADIUS)) < gate_m:
                return dict(out, valid=True, row=row, measured_surface=t, centre_range=t+RADIUS,
                            decision='RANGE_UNCONFIRMED_CONTRACT_INELIGIBLE')
        return out
    rays = v1.beams()[:, h]
    cosine = np.clip(rays @ unit, -1., 1.)
    disc = RADIUS**2 - column**2 * np.maximum(0., 1.-cosine**2)
    roots = column*cosine + np.sqrt(np.maximum(disc, 0.))
    support = measured_ok & (cosine > 0) & (disc >= 0) & (abs(roots-distance) < gate_m)
    if int(support.sum()) < 2:
        return out
    fitted = float(np.median(roots[support]))
    model, model_hit = _sphere_ranges(fitted*unit, rays)
    paired = measured_ok & model_hit
    if int(paired.sum()) < 2 or abs(fitted-distance) >= gate_m:
        return out
    if (measured_ok & ~model_hit & (abs(column - (fitted-RADIUS)) < gate_m)).any():
        out['decision'] = 'REJECT_EXTRA_NEAR_SURFACE'
        return out
    sphere_loss = float(np.max(abs(column[paired] - model[paired])))
    cosine_el = np.cos(v1.ELEVATIONS)
    horizontal = float(np.median(column[paired]*cosine_el[paired]))
    vertical_loss = float(np.max(abs(column[paired] - horizontal/cosine_el[paired])))
    margin = max(.001, 4*noise_bound_m)
    out.update(sphere_loss_m=sphere_loss, vertical_loss_m=vertical_loss,
               required_margin_m=margin, fitted_center_range=fitted)
    if sphere_loss > max(.001, 3*noise_bound_m) or vertical_loss-sphere_loss <= margin:
        out['decision'] = 'ABSTAIN_HYPOTHESES'
        return out
    if candidate == 'V2-B':
        if math.asin(min(1., RADIUS/fitted)) >= 2*math.pi/72:
            out['decision'] = 'ABSTAIN_AZIMUTH'
            return out
        for neighbor in ((h-1) % 72, (h+1) % 72):
            expected, reached = _sphere_ranges(fitted*unit, v1.beams()[:, neighbor])
            observed = scan[:, neighbor]
            valid = np.isfinite(observed) & (observed > 0) & (observed < FAR*.995)
            if (valid & ~reached & (abs(observed-(fitted-RADIUS)) < gate_m)).any():
                out['decision'] = 'REJECT_AZIMUTH_EXTENT'
                return out
    row = int(np.argmin(np.where(paired, column, np.inf)))
    return dict(out, valid=True, row=row, measured_surface=float(column[row]),
                centre_range=fitted, decision='ACCEPT_GEOMETRY')


def perturb_scan(scan, bound, sign=1):
    scan = np.asarray(scan, float)
    pattern = np.where(np.indices(scan.shape).sum(axis=0) % 2, -1., 1.) * sign
    return np.where(scan < FAR, np.clip(scan + bound*pattern, .000001, FAR), scan)


def geometries(root):
    """Predeclared geometry rows. This evaluator-only generator may read frozen GT/layout."""
    ranges = (1.16,1.5,2.,2.28,2.295,2.3,3.,6.)
    bearings = (0,20,45,90,-45,179)
    for r, el, b in itertools.product(ranges, np.arange(-10,20.01,.5), bearings):
        target = r*direction(b, el)
        scan, labels = v1.lidar_scan(sphere_center=target)
        yield {'set':'positive','r':r,'el':float(el),'b':b,'prediction':target,
               'target':target,'scan':scan,'labels':labels}
    for r, b in itertools.product(np.arange(1.5,11.51,.5), (0,20,45,90,180)):
        unit = direction(b, 0)
        center = (r+.3)*unit; center[2]=.5
        scan, labels = v1.lidar_scan([center], [[.3,.3,1.5]])
        h = int(np.argmin(abs(np.arctan2(np.sin(v1.ANGLES-math.radians(b)), np.cos(v1.ANGLES-math.radians(b))))))
        prediction = (scan[2,h]+.15)*unit
        yield {'set':'tall','r':float(r),'el':0.,'b':b,'prediction':prediction,'target':None,'scan':scan,'labels':labels}
    for r,b,width,el in itertools.product((1.16,1.5,2,2.28,2.3,3), (0,20,45,90), (.02,.06,.15,.3,.8), (-5,0,5,15)):
        center = (r+width/2)*direction(b,0); center[2]=.5
        scan, labels = v1.lidar_scan([center], [[width/2,width/2,1.5]])
        h = int(np.argmin(abs(np.arctan2(np.sin(v1.ANGLES-math.radians(b)), np.cos(v1.ANGLES-math.radians(b))))))
        row = int(np.argmin(abs(v1.ELEVATIONS-math.radians(el))))
        prediction = (scan[row,h]+.15)*direction(b,el)
        yield {'set':'narrow','r':r,'el':el,'b':b,'prediction':prediction,'target':None,'scan':scan,'labels':labels}
    for r,b,el,gap in itertools.product((1.16,1.5,2,2.28,2.3,3), (0,20,45,90), (-5,0,5,15), (.02,.05,.1,.3)):
        target = r*direction(b,el)
        center = ((r-RADIUS-gap)*math.cos(math.radians(el))+.12)*direction(b,0); center[2]=.5
        scan, labels = v1.lidar_scan([center], [[.12,.12,1.5]], sphere_center=target)
        yield {'set':'foreground','r':r,'el':el,'b':b,'prediction':target,'target':target,'scan':scan,'labels':labels,'gap':gap}
    frozen = Path(root).parent/'aerial_gym_simulator/results/eq1d_qualification_v1/EQ1D-R1/d2_scenes'
    meta = json.loads((frozen/'d2_meta.json').read_text())
    with np.load(frozen/'d2_renders.npz', allow_pickle=False) as z:
        for scene in ('N1','N2','N3','N5','N6','N7','N8'):
            i = meta['scenes'].index(scene)
            scan = z['level_scene__lidar_m'][i].reshape(4,72).astype(float)
            drone = z['drone_env_m'][i].astype(float)
            boxes = meta['layout'][i]
            hits = v1.ref.first_hits(drone, v1.beams(), [b['centre'] for b in boxes], [b['half'] for b in boxes], far_m=12.)
            if np.max(abs(hits['depth_m']-scan)) > 1e-4:
                raise ValueError('Frozen EQ1D scan/oracle mismatch: '+scene)
            b = {'N1':0,'N2':20,'N3':0,'N5':90,'N6':180,'N7':0,'N8':0}[scene]
            h = int(np.argmin(abs(np.arctan2(np.sin(v1.ANGLES-math.radians(b)),np.cos(v1.ANGLES-math.radians(b))))))
            prediction = (hits['depth_m'][2,h]+.15)*direction(b,0)
            yield {'set':'eq1d','scene':scene,'r':float(np.linalg.norm(prediction)),'el':0.,'b':b,
                   'prediction':prediction,'target':None,'scan':scan,'labels':hits['hit_class']}
