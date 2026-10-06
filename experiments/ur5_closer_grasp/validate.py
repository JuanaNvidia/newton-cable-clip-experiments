"""Audit recorded sequence, attachment integrity, seating and final retention."""
from pathlib import Path
from motion_io import load_motion,motion_sha256
import argparse,json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation as R
from scene import SOCKET_POSITIONS,SOCKET_YAWS,CLIP_ORIGINS,CLIP_YAWS,L
from clip_geometry import HOLE_BOTTOM,HOLE_TOP
from audit_geometry import retained_at_clips
from robot import poses as fk_poses,packed,JOINTS
root=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--motion',default='motion');a=p.parse_args();d=load_motion(root/(a.motion+'.npz'));q=d['poses'];ids=d['rod_ids'].reshape(6,-1);ds=float(d['segment_length']);tops=d['clip_top_bodies'];plugs=d['plug_bodies'];fps=int(d['fps'])
stages=[]
if a.motion=='motion' and (root/'recording_provenance.json').exists():
 provenance=json.loads((root/'recording_provenance.json').read_text())
 if provenance['motion_sha256']==motion_sha256(root/'motion.npz'):stages=provenance['segments']
stage_rod_gaps={};settled_rod_gap=0.;phase_rod_gaps={};rodgap_frame=0;rodgap=0.;attachgap=0.;hingegap=0.;fkerr=0.;tablepenetration=0.;final=[];latch_events=[]
for f in range(0,len(q),6):
 bodies=q[f];rods=bodies[ids];v=R.from_quat(rods[:,:,3:].reshape(-1,4)).apply(np.tile([0,0,ds/2],(ids.size,1))).reshape(6,-1,3);aa=rods[:,:,:3]-v;bb=rods[:,:,:3]+v
 value=float(np.linalg.norm(bb[:,:-1]-aa[:,1:],axis=2).max());phase=str(d['phases'][f]);phase_rod_gaps[phase]=max(phase_rod_gaps.get(phase,0.),value*1000)
 if value>rodgap:rodgap=value;rodgap_frame=f
 if f>=len(q)-120:settled_rod_gap=max(settled_rod_gap,value*1000)
 for stage_index,stage in enumerate(stages):
  if stage['start_frame']<=f<stage['end_frame_exclusive']:
   label=f"{stage_index+1}: {stage['controller']} ({stage['iterations']} iterations)";stage_rod_gaps[label]=max(stage_rod_gaps.get(label,0.),value*1000);break
 tablepenetration=max(tablepenetration,float(np.maximum(0,.002-np.minimum(aa[:,:,2],bb[:,:,2])).max()))
 for c in range(6):
  for k,body in enumerate(plugs):
   target=R.from_quat(bodies[body,3:]).apply(d['plug_anchors'][2*c+k])+bodies[body,:3]
   attachgap=max(attachgap,float(np.linalg.norm(target-(aa[c,0] if k==0 else bb[c,-1]))))
 for k,top in enumerate(tops):
  actual=R.from_quat(bodies[top,3:]).apply(HOLE_TOP)+bodies[top,:3];target=CLIP_ORIGINS[k]+R.from_euler('z',CLIP_YAWS[k]).apply(HOLE_BOTTOM);hingegap=max(hingegap,float(np.linalg.norm(actual-target)))
 if f>=len(q)-120:final.append(retained_at_clips(bodies,d['rod_ids'],ds,tops))
from robot import theta_for_gap
for f in range(0,len(q),60):
 truth=packed(fk_poses(d['joint_positions'][f],theta_for_gap(d['jaw_width'][f])));fkerr=max(fkerr,float(np.linalg.norm(truth[:,:3]-q[f,int(d['robot_start']):,:3],axis=1).max()))
latch=np.asarray(d['latch_history']);prev=np.zeros(2,dtype=bool)
for f,state in enumerate(latch):
 for k in np.where(state&~prev)[0]:
  b=q[f,plugs[k]];latch_events.append(dict(connector=int(k+1),time=f/fps,position_error_mm=float(np.linalg.norm(b[:3]-SOCKET_POSITIONS[k])*1000),angle_error_degrees=float(np.degrees((R.from_euler('z',SOCKET_YAWS[k]).inv()*R.from_quat(b[3:])).magnitude()))))
 prev=state
# Check every recorded pose in the final two-second retention window.
final=[retained_at_clips(bodies,d['rod_ids'],ds,tops) for bodies in q[-120:]]
retained=np.all(final,axis=0) if final else np.zeros((3,6),bool)
seated_errors=[]
for k,body in enumerate(plugs):
 seated_errors.append(float(np.max(np.linalg.norm(q[-120:,body,:3]-SOCKET_POSITIONS[k],axis=1))*1000))
limits=[j.find('limit') for j in JOINTS if j.get('type')!='fixed'];lower=np.array([float(x.get('lower')) for x in limits]);upper=np.array([float(x.get('upper')) for x in limits]);velocity=np.array([float(x.get('velocity')) for x in limits]);joints=d['joint_positions'];speed=np.max(abs(np.diff(joints,axis=0))*fps,axis=0)
contact_counts=np.asarray(d['contact_counts']) if 'contact_counts' in d else np.full(len(q),-1);known_contacts=contact_counts[contact_counts>=0]
checks=dict(recorded_contact_counts_below_capacity=bool(len(known_contacts) and known_contacts.max()<150000),robot_joint_limits=bool(((joints>=lower-1e-6)&(joints<=upper+1e-6)).all()),robot_joint_velocity_limits=bool((speed<=velocity+1e-3).all()),finite=bool(np.isfinite(q).all()),length_762_mm=abs(ids.shape[1]*ds-L)<1e-8,both_connectors_started_unlatched=not bool(latch[0].any()),both_connectors_seated=bool(latch[-1].all()) and max(seated_errors)<1.,sequential_connector_latches=len(latch_events)==2 and latch_events[0]['connector']==1 and latch_events[1]['connector']==2,all_cables_in_all_three_clips=bool(retained.all()),cable_joints_within_point2_mm=rodgap<.0002,end_attachments_within_point2_mm=attachgap<.0002,hinges_within_point05_mm=hingegap<.00005,robot_matches_fk=fkerr<1e-5,cable_table_overlap_below_point2_mm=tablepenetration<.0002)
# Separate geometric channel occupancy from fit under a fully closed lid.
# This changes only an audit copy of the lid transforms, never the recording.
closed_envelope=[]
for bodies in q[-120::6]:
 audit=bodies.copy()
 for origin,yaw,top in zip(CLIP_ORIGINS,CLIP_YAWS,tops):
  rotation=R.from_euler('z',yaw);audit[top,:3]=origin+rotation.apply(HOLE_BOTTOM-HOLE_TOP);audit[top,3:]=rotation.as_quat()
 closed_envelope.append(retained_at_clips(audit,d['rod_ids'],ds,tops))
closed_counts=np.sum(np.all(closed_envelope,axis=0),axis=1).tolist()
r=dict(motion_sha256=motion_sha256(root/(a.motion+'.npz')),passed=all(checks.values()),checks=checks,measurements=dict(closed_lid_envelope_counts=closed_counts,max_recorded_contact_count=int(known_contacts.max()) if len(known_contacts) else None,contact_count_recording_coverage=float(len(known_contacts)/len(q)),max_joint_velocity_rad_s=speed.tolist(),joint_velocity_limits_rad_s=velocity.tolist(),final_lid_angles_deg=[float((R.from_euler('z',yaw).inv()*R.from_quat(q[-1,top,3:])).as_euler('xyz',degrees=True)[1]) for yaw,top in zip(CLIP_YAWS,tops)],final_retained_per_clip=retained.tolist(),final_retained_counts=np.sum(retained,axis=1).tolist(),latch_events=latch_events,final_connector_position_errors_mm=seated_errors,max_cable_joint_gap_mm=rodgap*1000,max_cable_joint_gap_frame=rodgap_frame,max_cable_joint_gap_by_phase_mm=phase_rod_gaps,max_cable_joint_gap_by_recording_segment_mm=stage_rod_gaps,final_two_seconds_max_cable_joint_gap_mm=settled_rod_gap,max_connector_attachment_gap_mm=attachgap*1000,max_hinge_anchor_error_mm=hingegap*1000,max_robot_fk_error_mm=fkerr*1000,max_capsule_table_overlap_mm=tablepenetration*1000),limitations='Geometric and sampled checks; strict clip, finger and robot-clearance reports are separate; not a continuous collision or force validation.')
(root/('validation.json' if a.motion=='motion' else a.motion+'_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
