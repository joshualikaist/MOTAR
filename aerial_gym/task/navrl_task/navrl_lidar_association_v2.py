"""Opt-in C-v2 sphere-versus-vertical hypothesis, sensor/track inputs only.

V1 accepted measurements, range-only covariance and clock updates remain unchanged.
New geometry decisions are bounded to the preregistered precision of the simulator's LiDAR.
"""
import math
import torch
from aerial_gym.task.navrl_task.navrl_sensor_lineage import range_only_lidar_association


def _median(values, supported):
    """Median including the average of the middle pair, like a scalar statistical median."""
    ordered=values.masked_fill(~supported,math.inf).sort(dim=1).values
    count=supported.sum(dim=1)
    lo=((count-1).clamp(min=0)//2).view(-1,1)
    hi=(count//2).view(-1,1)
    result=(ordered.gather(1,lo)+ordered.gather(1,hi)).squeeze(1)*.5
    return torch.where(count>0,result,torch.zeros_like(result))


def range_only_lidar_association_v2(lidar_ranges,lidar_angles,lidar_elevations,predicted_rel,camera_visible,
                                  active,gate_m,*,lidar_far_m,lidar_target_radius=.20,
                                  noise_bound_m=.0001):
    """Add V2-A acceptances only; no semantic identity, target GT or clock input is accepted."""
    if not math.isfinite(noise_bound_m) or noise_bound_m<0:
        raise ValueError('noise bound must be finite and nonnegative')
    base=range_only_lidar_association(lidar_ranges,lidar_angles,lidar_elevations,predicted_rel,
                                     camera_visible,active,gate_m,lidar_far_m=lidar_far_m,
                                     lidar_target_radius=lidar_target_radius)
    n,v,h=lidar_ranges.shape
    rows=torch.arange(n,device=lidar_ranges.device)
    column=lidar_ranges[rows,:,base['column']]
    finite=torch.isfinite(column)&(column>0)&(column<lidar_far_m*.995)
    clean=torch.where(finite,column,torch.full_like(column,lidar_far_m))
    distance=predicted_rel.norm(dim=1).clamp(min=1e-6)
    unit=predicted_rel/distance[:,None]
    az=lidar_angles[base['column']]
    ce=torch.cos(lidar_elevations)[None,:]
    se=torch.sin(lidar_elevations)[None,:]
    ray=torch.stack(torch.broadcast_tensors(ce*torch.cos(az)[:,None],ce*torch.sin(az)[:,None],
                                           se+torch.zeros_like(az)[:,None]),dim=-1)
    cosine=(ray*unit[:,None,:]).sum(dim=2).clamp(-1,1)
    discriminant=lidar_target_radius**2-clean.square()*(1-cosine.square()).clamp(min=0)
    root=clean*cosine+discriminant.clamp(min=0).sqrt()
    support=finite&(cosine>0)&(discriminant>=0)&((root-distance[:,None]).abs()<gate_m[:,None])
    fitted=_median(root,support)
    center=fitted[:,None]*unit
    projection=(ray*center[:,None,:]).sum(dim=2)
    discr=projection.square()-(fitted.square()-lidar_target_radius**2)[:,None]
    model=projection-discr.clamp(min=0).sqrt()
    reached=(discr>=0)&(model>0)&(model<lidar_far_m)
    paired=finite&reached
    extra=(finite&~reached&((clean-(fitted-lidar_target_radius)[:,None]).abs()<gate_m[:,None])).any(dim=1)
    sphere_loss=(clean-model).abs().masked_fill(~paired,0).amax(dim=1)
    horizontal=_median(clean*ce,paired)
    plane_loss=(clean-horizontal[:,None]/ce).abs().masked_fill(~paired,0).amax(dim=1)
    recovered=(~base['valid']&active&~camera_visible&base['conclusive']
               &(support.sum(dim=1)>=2)&(paired.sum(dim=1)>=2)&~extra
               &((fitted-distance).abs()<gate_m)
               &(sphere_loss<=max(.001,3*noise_bound_m))
               &((plane_loss-sphere_loss)>max(.001,4*noise_bound_m))
               &torch.isfinite(predicted_rel).all(dim=1))
    chosen=clean.masked_fill(~paired,math.inf).argmin(dim=1)
    measured=clean[rows,chosen]
    result=dict(base)
    result['valid']=base['valid']|recovered
    result['row']=torch.where(recovered,chosen,base['row'])
    result['measured_surface']=torch.where(recovered,measured,base['measured_surface'])
    result['centre_range']=torch.where(recovered,fitted,base['centre_range'])
    result['measurement_vehicle']=torch.where(recovered[:,None],center,base['measurement_vehicle'])
    result['v2_recovered']=recovered
    result['v2_sphere_loss']=sphere_loss
    result['v2_vertical_loss']=plane_loss
    result['v2_support_rows']=support.sum(dim=1)
    result['v2_abstain']=(~result['valid'])&active&~camera_visible
    return result
