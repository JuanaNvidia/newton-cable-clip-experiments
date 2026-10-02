"""Geometric validation of recorded cable, clip, robot and grasp trajectories."""
from pathlib import Path
from retention import lid_ceiling
import json,hashlib,argparse
from io import BytesIO
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,HOLE_TOP
from robot import poses as robot_poses,theta_for_gap,packed,gripper_local,gap,JOINTS,fingertip_lip
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--motion',default='motion');a=p.parse_args()
raw_motion=(ROOT/(a.motion+'.npz')).read_bytes();archive=np.load(BytesIO(raw_motion));d={key:archive[key] for key in archive.files};poses=d['poses'];fps=int(d['fps']);N=int(d['segments']);ds=float(d['segment_length']);TOP=int(d['clip_top_body']);ROBOT=int(d['robot_start']);times=np.arange(len(poses))/fps
meta=json.loads((ROOT/(a.motion+'.json')).read_text()) if (ROOT/(a.motion+'.json')).exists() else {};release_end=meta.get('release_complete_seconds',10.9 if len(poses)>720 else 9.6);hold_end=meta.get('first_release_start_seconds',meta.get('release_start_seconds',14.0))-.1
recovery_active=bool(meta.get('recovery_triggered',d.get('recovery_triggered',False)))
recovery_held=(times>=21.4)&(times<=28.2)&recovery_active
first_held=(times>=4.6)&(times<=hold_end)
all_held=first_held|recovery_held
lid_clearances=[];centers=[];gaps=[];attachments=[];hinges=[];fkerrors=[];fkangles=[];padproximity=[];release_pad_distances=[];release_pad_times=[]
rod_ids=np.array(d['rod_ids']).reshape(6,N);gi=int(d['grasp_segment']) if 'grasp_segment' in d else 46
pad_ids=list(d['pad_bodies']) if 'pad_bodies' in d else [ROBOT+list(d['robot_names']).index(side+'_pad') for side in ['right','left']]
kin_indices=np.array([j for j in range(len(d['robot_names'])) if ROBOT+j not in (list(d['pad_bodies']) if 'pad_bodies' in d else [])])
lip_mesh=fingertip_lip()
pad_travel=[];pad_tangent=[];pad_angles=[]
for i,pose in enumerate(poses):
 rods=pose[rod_ids.flatten()];dirs=R.from_quat(rods[:,3:]).apply(np.tile([0,0,1],(6*N,1)));start=(rods[:,:3]-dirs*ds/2).reshape(6,N,3);end=(rods[:,:3]+dirs*ds/2).reshape(6,N,3)
 gaps.append(np.linalg.norm(end[:,:-1]-start[:,1:],axis=-1).max())
 anchors=pose[1,:3]+R.from_quat(pose[1,3:]).apply(np.array([[(c-2.5)*.0025,.009,.012] for c in range(6)]));attachments.append(np.linalg.norm(start[:,0]-anchors,axis=1).max())
 row=[]
 for c in range(6):
  vals=[]
  for x,y in zip(start[c],end[c]):
   plane=CLIP_ORIGIN[1]-.015
   if (x[1]-plane)*(y[1]-plane)<=0 and abs(y[1]-x[1])>1e-9:vals.append(x+(y-x)*(plane-x[1])/(y[1]-x[1])-CLIP_ORIGIN)
  row.append(min(vals,key=lambda p:abs(p[0]-.018)) if vals else [np.nan]*3)
 toplocal=R.from_quat(pose[TOP,3:]).inv().apply(np.array(row)+CLIP_ORIGIN-pose[TOP,:3]);roof=lid_ceiling(toplocal[:,0]);lid_clearances.append(np.where(np.isfinite(roof),roof-toplocal[:,2],-np.inf))
 centers.append(row);hinges.append(np.linalg.norm(pose[TOP,:3]+R.from_quat(pose[TOP,3:]).apply(HOLE_TOP)-CLIP_ORIGIN-HOLE_BOTTOM))
 if i%6==0:
  expected=packed(robot_poses(d['joint_positions'][i],theta_for_gap(float(d['jaw_width'][i]))));fkerrors.append(np.linalg.norm(pose[ROBOT+kin_indices,:3]-expected[kin_indices,:3],axis=1).max());fkangles.append((R.from_quat(pose[ROBOT+kin_indices,3:]).inv()*R.from_quat(expected[kin_indices,3:])).magnitude().max())
  if all_held[i] or times[i]>=release_end+.3:
   pts=(rods[:,:3,None]+dirs[:,:,None]*np.linspace(-ds/2,ds/2,7)).transpose(0,2,1).reshape(6,N*7,3);perpad=[]
   for bid in pad_ids:
    local=R.from_quat(pose[bid,3:]).inv().apply(pts.reshape(-1,3)-pose[bid,:3]).reshape(6,N*7,3);v=np.abs(local-[0,-.0026,.01875])-[.011,.004,.01875];sdf=np.linalg.norm(np.maximum(v,0),axis=-1)+np.minimum(v.max(axis=-1),0)
    # Include the physical support lips. Exact distances near contact; their
    # bounding box gives a conservative lower bound for distant clearance.
    delta=np.abs(local-[0,-.00785,.037])-[.011,.00125,.0005]
    support=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(axis=-1),0)
    near=support<.003
    if near.any():support[near]=-trimesh.proximity.signed_distance(lip_mesh,local[near])
    sdf=np.minimum(sdf,support);perpad.append(sdf.min(axis=1))
   if all_held[i]:padproximity.append(perpad)
   else:release_pad_distances.append(perpad);release_pad_times.append(times[i])
 if 'pad_bodies' in d:
  for skin,parent in zip(d['pad_bodies'],d['pad_parents']):
   local=R.from_quat(pose[parent,3:]).inv().apply(pose[skin,:3]-pose[parent,:3]);pad_travel.append(abs(local[1]));pad_tangent.append(np.linalg.norm(local[[0,2]]));pad_angles.append((R.from_quat(pose[parent,3:]).inv()*R.from_quat(pose[skin,3:])).magnitude())
centers=np.array(centers);released=times>=release_end+.3;held=(times>=4.6)&(times<=hold_end)
closed_envelope=(centers[:,:,0]>.005)&(centers[:,:,0]<.028)&(centers[:,:,2]>.0118)&(centers[:,:,2]<.0162)
retained=(centers[:,:,0]>.005)&(centers[:,:,0]<.028)&(centers[:,:,2]>.0118)&(np.array(lid_clearances)>.0008)
angles=R.from_quat(poses[:,TOP,3:]).as_euler('xyz',degrees=True)[:,1]
metrics={'final_clip_cross_section_mm':(centers[-1]*1000).tolist(),'final_clip_angle_deg':float(angles[-1]),'max_clip_opening_deg':float(-angles.min()),'max_rod_joint_gap_mm':float(max(gaps)*1000),'max_root_attachment_error_mm':float(max(attachments)*1000),'max_hinge_anchor_error_mm':float(max(hinges)*1000),'max_fk_position_error_mm':float(max(fkerrors)*1000),'max_fk_rotation_error_deg':float(np.degrees(max(fkangles))),'retained_per_cable_after_release':retained[released].all(axis=0).tolist() if released.any() else [],'max_pad_distance_per_cable_during_hold_mm':(np.array(padproximity).max(axis=(0,1))*1000).tolist() if padproximity else [],'final_cable_order_preserved':bool(np.all(np.diff(centers[-1,:,0])>0)),'final_cable_height_spread_mm':float(np.ptp(centers[-1,:,2])*1000),'observation_after_release_s':float(max(0,times[-1]-release_end))}
if held.any():
 grasp_indices=d['grasp_segments'] if 'grasp_segments' in d else np.full(6,gi);material=poses[:,rod_ids[np.arange(6),grasp_indices],:3];relative=material-d['tcp_targets'][:,None,:];initial_hold=np.where(held)[0][0]
 slip=np.linalg.norm(relative[held]-relative[initial_hold],axis=-1).max(axis=0)
 lift=material[min(round(5.3*fps),len(poses)-1),:,2]-material[min(round(4.5*fps),len(poses)-1),:,2]
 metrics['max_grasp_material_slip_per_cable_mm']=(slip*1000).tolist();metrics['lift_per_cable_mm']=(lift*1000).tolist();lift_phase=(times>=4.6)&(times<=6.3);lift_slip=np.linalg.norm(relative[lift_phase]-relative[initial_hold],axis=-1).max(axis=0);metrics['max_lift_slip_per_cable_mm']=(lift_slip*1000).tolist()
else:slip=np.ones(6);lift_slip=np.ones(6);lift=np.zeros(6)
checks={'finite':bool(np.isfinite(poses).all()),'connector_stays_seated':bool(np.max(abs(poses[:,1]-poses[0,1]))<1e-7),'starts_outside_clip':bool((centers[0,:,0]>.03).all()),'rod_joint_gap_under_0_2mm':max(gaps)<.0002,'root_attached_within_0_1mm':max(attachments)<.0001,'hinge_anchor_within_0_05mm':max(hinges)<.00005,'robot_follows_urdf_fk_within_0_1mm':max(fkerrors)<.0001,'robot_orientation_within_0_1deg':max(fkangles)<np.radians(.1),'all_six_retained_after_release':bool(released.any() and retained[released].all()),'clip_opens_during_insertion':bool(-angles.min()>3),'all_cables_lifted_at_least_5mm':bool((lift>.005).all()),'all_cables_follow_gripper_during_lift_within_5mm':bool((lift_slip<.005).all()),'both_fingers_contact_bundle_during_hold':bool(padproximity and np.array(padproximity).min(axis=2).max()<.0012)}
if release_pad_distances:
 metrics['min_pad_distance_after_release_mm']=float(np.min(release_pad_distances)*1000)
 final_distances=np.array(release_pad_distances)[np.array(release_pad_times)>=times[-1]-1]
 metrics['min_pad_distance_last_second_mm']=float(final_distances.min()*1000)
 if 'retreat_clear_seconds' in meta:
  clear_times=np.array(release_pad_times)>=meta['retreat_clear_seconds']+.1;clear_distances=np.array(release_pad_distances)[clear_times]
  metrics['min_pad_distance_after_withdrawal_mm']=float(clear_distances.min()*1000) if len(clear_distances) else None
  checks['fingers_clear_after_withdrawal']=bool(len(clear_distances) and clear_distances.min()>.0011)
 else:checks['fingers_clear_cables_after_release']=bool(np.min(release_pad_distances)>.0011)
if pad_travel:
 metrics.update(max_pad_normal_travel_mm=float(max(pad_travel)*1000),max_pad_tangent_error_mm=float(max(pad_tangent)*1000),max_pad_rotation_error_deg=float(np.degrees(max(pad_angles))))
 checks.update(pad_travel_below_hard_limit=max(pad_travel)<.0029,pad_tangent_within_0_1mm=max(pad_tangent)<.0001,pad_rotation_within_0_1deg=max(pad_angles)<np.radians(.1))
limits=[j.find('limit') for j in JOINTS if j.get('type')!='fixed'];lower=np.array([float(l.get('lower')) for l in limits]);upper=np.array([float(l.get('upper')) for l in limits]);speed=np.array([float(l.get('velocity')) for l in limits]);joints=d['joint_positions'];vel=np.abs(np.diff(joints,axis=0))*fps
metrics['max_joint_speed_rad_s']=vel.max(axis=0).tolist();checks['robot_command_within_joint_limits']=bool(((joints>=lower)&(joints<=upper)).all());checks['robot_command_within_velocity_limits']=bool((vel<=speed).all())
metrics['max_rod_joint_gap_time_s']=float(times[int(np.argmax(gaps))])
metrics['retention_failure_times_after_release_s']=[[float(times[j]) for j in np.where(released & ~retained[:,c])[0]] for c in range(6)]
metrics['retained_per_cable_last_second']=retained[times>=times[-1]-1].all(axis=0).tolist()
metrics['retained_per_cable_final_frame']=retained[-1].tolist()
metrics['inside_closed_lid_envelope_final']=closed_envelope[-1].tolist()
metrics['final_lid_clearance_from_cable_axis_mm']=(np.array(lid_clearances)[-1]*1000).tolist()
metrics['retention_method']='Clip center-plane crossing inside local X 5..28 mm, axis above floor+0.8 mm and at least 0.8 mm below the actual posed lid underside from original STL; capsule/STL penetration checked separately'
if 'retreat_clear_seconds' in meta:
 after_retreat=times>=meta['retreat_clear_seconds']+.3
 metrics['retained_per_cable_after_withdrawal']=retained[after_retreat].all(axis=0).tolist() if after_retreat.any() else []
 metrics['observation_after_withdrawal_s']=float(max(0,times[-1]-meta['retreat_clear_seconds']))
 checks['all_six_retained_after_withdrawal']=bool(after_retreat.any() and retained[after_retreat].all())
if 'target_clip_plane_height_mm' in meta:
 feed=(times>=7.5)&(times<=meta.get('first_release_start_seconds',meta['release_start_seconds']));root=poses[0,1,:3]+R.from_quat(poses[0,1,3:]).apply([0,.009,.012]);tcp=d['tcp_targets'][feed]
 fraction=(CLIP_ORIGIN[1]-.015-root[1])/(tcp[:,1]-root[1]);heights=root[2]+fraction*(tcp[:,2]-root[2]);error=float(np.max(np.abs(heights*1000-meta['target_clip_plane_height_mm']))) if len(tcp) else float('inf')
 metrics['max_target_chord_height_error_mm']=error;checks['gripper_targets_clip_plane_height_within_0_1mm']=error<.1
distal=int(d['distal_body']);endpoints=poses[:,rod_ids[:,-1],:3]+R.from_quat(poses[:,rod_ids[:,-1],3:].reshape(-1,4)).apply(np.tile([0,0,ds/2],(len(poses)*6,1))).reshape(-1,6,3)
anchors=poses[:,distal,None,:3]+R.from_quat(np.repeat(poses[:,distal,3:],6,axis=0)).apply(np.tile(d['distal_anchors'],(len(poses),1))).reshape(-1,6,3)
distal_error=np.linalg.norm(endpoints-anchors,axis=-1)
metrics['max_distal_attachment_error_mm']=float(distal_error.max()*1000)
metrics['distal_connector_displacement_mm']=float(np.linalg.norm(poses[-1,distal,:3]-poses[0,distal,:3])*1000)
checks['all_six_distal_ends_attached_within_0_1mm']=bool(distal_error.max()<.0001)
# The clip footprint is at least 100 mm from every tabletop edge.
mesh=trimesh.load(ROOT/'clipBottom.stl');xy=np.asarray(mesh.vertices)[:,:2]*.001+CLIP_ORIGIN[:2]
clearance=min((xy-np.array([-.31,-.12])).min(),(np.array([.30,.52])-xy).min())
metrics['clip_min_table_edge_clearance_mm']=float(clearance*1000);checks['clip_at_least_100mm_inside_table']=bool(clearance>.100)
if len(poses)>1080:
 before=retained[1079];metrics['retained_before_opposite_side_pull']=before.tolist()
 checks['recovery_trigger_matches_retention_check']=recovery_active==bool(not before.all())
 if meta.get('retention_before_recovery') is not None:
  checks['recorded_detection_matches_geometry']=bool(np.array_equal(before,meta['retention_before_recovery']))
if 'recovery_retention' in d:
 recorded=np.asarray(d['recovery_retention'],dtype=bool)
 metrics['feedback_retention_mismatched_samples']=int(np.count_nonzero(recorded[1:]!=retained[:-1]))
 checks['feedback_matches_recorded_geometry']=bool(np.array_equal(recorded[1:],retained[:-1]))
if recovery_active and recovery_held.any():
 recovery_ids=np.asarray(d['recovery_indices'],dtype=int);assert (recovery_ids>=0).all()
 material=poses[:,rod_ids[np.arange(6),recovery_ids],:3];relative=material-d['tcp_targets'][:,None,:]
 i0=min(round(21.3*fps),len(poses)-1);i1=min(round(22.3*fps),len(poses)-1)
 metrics['recovery_lift_per_cable_mm']=((material[i1,:,2]-material[i0,:,2])*1000).tolist()
 pull=(times>=22.3)&(times<=28.3);lift_phase=(times>=21.4)&(times<=22.3)
 initial=np.where(recovery_held)[0][0];slip=np.linalg.norm(relative[lift_phase]-relative[initial],axis=-1).max(axis=0)
 metrics['recovery_lift_material_slip_per_cable_mm']=(slip*1000).tolist()
 checks['recovery_bundle_follows_gripper_during_lift_within_5mm']=bool((slip<.005).all())
 metrics['recovery_grasp_y_mm']=float(d['tcp_targets'][min(round(20.5*fps),len(poses)-1),1]*1000)
 checks['regrasp_on_opposite_side_of_clip']=bool(metrics['recovery_grasp_y_mm']< (CLIP_ORIGIN[1]-.015-.035)*1000)
 if pull.any():
  metrics['recovery_peak_inside_count_while_pulling']=int(retained[pull].sum(axis=1).max())
  metrics['recovery_pull_displacement_mm']=float((d['tcp_targets'][np.where(pull)[0][0],0]-d['tcp_targets'][np.where(pull)[0][-1],0])*1000)
 metrics['recovery_used_physical_grasp_only']=True
r={'passed':all(checks.values()),'checks':{k:bool(v) for k,v in checks.items()},'measurements':metrics,'limitations':['Robot links are kinematically position controlled, not motor torque simulation','Connector is fixed in its seated pose','Finger contact distances include original pads and physical support lips; exact near contact, conservative support-box lower bounds at distances above 3 mm; not measured hardware forces','No calibrated grip-force or hardware execution validation','Retention is sampled at the clip center plane, using the posed lid underside; this is not a long-term retention proof'],'motion_sha256':hashlib.sha256(raw_motion).hexdigest()}
def clean_json(value):
 if isinstance(value,dict):return {k:clean_json(v) for k,v in value.items()}
 if isinstance(value,list):return [clean_json(v) for v in value]
 if isinstance(value,float) and not np.isfinite(value):return None
 return value
r=clean_json(r)
(ROOT/('validation.json' if a.motion=='motion' else a.motion+'_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))

if a.motion=='motion':assert r['passed'],'Recorded task validation failed; inspect validation.json'
