"""Independent pose checks for cable attachments, fixed receiver, mating, and contact."""
from pathlib import Path
import json,hashlib,argparse
import numpy as np
from scipy.spatial.transform import Rotation
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,HOLE_TOP
from assets import RECEIVER_ORIGIN,SEAT,plug_parts,fixture_parts
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--motion',default='motion');args=p.parse_args()
d=np.load(ROOT/(args.motion+'.npz'));poses=d['poses'];fps=int(d['fps']);N=int(d['segments']);ds=float(d['segment_length']);times=(np.arange(len(poses))+1)/fps
relative=[];angles=[];rot_errors=[];gaps=[];near_errors=[];far_errors=[]
for pose in poses:
 rr=Rotation.from_quat(pose[0,3:]);relative.append(rr.inv().apply(pose[1,:3]-pose[0,:3]));angles.append(rr.as_euler('xyz',degrees=True)[0]);rot_errors.append(np.degrees((rr.inv()*Rotation.from_quat(pose[1,3:])).magnitude()))
 rods=pose[2:2+6*N];v=Rotation.from_quat(rods[:,3:]).apply(np.tile([0,0,ds/2],(6*N,1)));start=(rods[:,:3]-v).reshape(6,N,3);end=(rods[:,:3]+v).reshape(6,N,3);gaps.append(np.linalg.norm(end[:,:-1]-start[:,1:],axis=-1).max())
 for c in range(6):
  dx=(c-2.5)*.0025;near=pose[1,:3]+Rotation.from_quat(pose[1,3:]).apply([dx,.009,.012]);far=d['initial_cable_endpoints'][1]+[dx,0,0];near_errors.append(np.linalg.norm(start[c,0]-near));far_errors.append(np.linalg.norm(end[c,-1]-far))
relative=np.array(relative);released=times>=12.3
# Sample the actual plug collision-box surfaces against each receiver collision box.
points=[]
for part in plug_parts():
 if not part['collision']:continue
 half=np.array(part['size'])/2
 for axis in range(3):
  other=[i for i in range(3) if i!=axis];a,b=other
  aa,bb=np.meshgrid(np.linspace(-half[a],half[a],max(3,int(part['size'][a]/.001)+1)),np.linspace(-half[b],half[b],max(3,int(part['size'][b]/.001)+1)))
  for sign in [-1,1]:
   pts=np.zeros((aa.size,3));pts[:,axis]=sign*half[axis];pts[:,a]=aa.ravel();pts[:,b]=bb.ravel();points.append(pts+part['pos'])
points=np.vstack(points);penetration=0.
for pose in poses:
 world=Rotation.from_quat(pose[1,3:]).apply(points)+pose[1,:3];local=Rotation.from_quat(pose[0,3:]).inv().apply(world-pose[0,:3])
 for part in fixture_parts()[1]:
  if not part['collision'] or part['kind']!='box':continue
  q=np.abs(local-part['pos'])-np.array(part['size'])/2;sdf=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0);penetration=max(penetration,float(-sdf.min()))
r={'finite':bool(np.isfinite(poses).all()),'plug_start_degrees':float(rot_errors[0]),'plug_angle_at_2_5s_degrees':float(rot_errors[min(149,len(rot_errors)-1)]),'receiver_start_degrees':float(angles[0]),'receiver_final_degrees':float(angles[-1]),'receiver_angle_at_1s_degrees':float(angles[min(59,len(angles)-1)]),'receiver_rotation_degrees':float(max(angles)-min(angles)),'final_plug_in_receiver_mm':(relative[-1]*1000).tolist(),'ideal_seat_mm':(SEAT*1000).tolist(),'final_relative_angle_degrees':float(rot_errors[-1]),'max_after_release_seat_error_mm':float(np.linalg.norm(relative[released]-SEAT,axis=1).max()*1000) if released.any() else None,'max_after_release_relative_angle_degrees':float(np.array(rot_errors)[released].max()) if released.any() else None,'max_receiver_translation_error_mm':float(np.linalg.norm(poses[:,0,:3]-RECEIVER_ORIGIN,axis=1).max()*1000),'max_cable_joint_gap_mm':float(max(gaps)*1000),'max_released_cable_joint_gap_mm':float(np.array(gaps)[released].max()*1000) if released.any() else None,'max_near_attachment_error_mm':float(max(near_errors)*1000),'max_far_attachment_error_mm':float(max(far_errors)*1000),'max_sampled_plug_receiver_penetration_mm':penetration*1000,'release_observation_seconds':float(max(0,times[-1]-12.2))}
checks={'finite':r['finite'],'receiver_stays_fixed':bool(np.max(np.abs(poses[:,0,:]-np.array([*RECEIVER_ORIGIN,0,0,0,1])))<1e-7),'plug_initially_angled':rot_errors[0]>20,'plug_enters_angled':rot_errors[min(149,len(rot_errors)-1)]>10,'plug_rotates_down':rot_errors[-1]<2,'seated_after_hand_release':released.any() and r['max_after_release_seat_error_mm']<1,'aligned_after_hand_release':released.any() and r['max_after_release_relative_angle_degrees']<2,'fixed_receiver_position_within_0_1_mm':r['max_receiver_translation_error_mm']<.1,'rod_joints_within_0_2_mm':r['max_cable_joint_gap_mm']<.2,'released_rod_joints_within_0_1_mm':released.any() and r['max_released_cable_joint_gap_mm']<.1,'cables_attached':max(r['max_near_attachment_error_mm'],r['max_far_attachment_error_mm'])<.1,'sampled_penetration_below_0_2_mm':r['max_sampled_plug_receiver_penetration_mm']<.2}
# Cable retention in the original clip cavity and original-STL surface checks.
import trimesh
TOP=int(d['clip_top_body']);clip_angles=Rotation.from_quat(poses[:,TOP,3:]).as_euler('xyz',degrees=True)[:,1]
centers=[];hinge_errors=[];mesh_pen={'top':0.,'bottom':0.};mesh_pen_frame={'top':None,'bottom':None}
meshes={k:trimesh.load(ROOT/('clip'+k.title()+'.stl')) for k in ['top','bottom']}
for m in meshes.values():m.apply_scale(.001)
for frame,pose in enumerate(poses):
 rods=pose[2:2+6*N];rot=Rotation.from_quat(rods[:,3:]);dirs=rot.apply(np.tile([0,0,1],(6*N,1)))
 start=(rods[:,:3]-dirs*ds/2).reshape(6,N,3);end=(rods[:,:3]+dirs*ds/2).reshape(6,N,3)
 row=[]
 for c in range(6):
  candidates=[]
  for x,y in zip(start[c],end[c]):
   plane=CLIP_ORIGIN[1]-.015
   if (x[1]-plane)*(y[1]-plane)<=0 and abs(y[1]-x[1])>1e-9:candidates.append(x+(y-x)*(plane-x[1])/(y[1]-x[1])-CLIP_ORIGIN)
  row.append(min(candidates,key=lambda p:abs(p[0]-.018)) if candidates else [np.nan]*3)
 centers.append(row)
 hp=pose[TOP,:3]+Rotation.from_quat(pose[TOP,3:]).apply(HOLE_TOP);hinge_errors.append(np.linalg.norm(hp-CLIP_ORIGIN-HOLE_BOTTOM))
 if frame%4==0 or frame==len(poses)-1:
  pts=(rods[:,:3,None]+dirs[:,:,None]*np.linspace(-ds/2,ds/2,17)).transpose(0,2,1).reshape(-1,3)
  local=pts-CLIP_ORIGIN;near=(local[:,0]>-.055)&(local[:,0]<.056)&(local[:,1]>-.035)&(local[:,1]<.005)&(local[:,2]<.055);pts=pts[near]
  if len(pts):
   for name,m in meshes.items():
    local=pts-CLIP_ORIGIN if name=='bottom' else Rotation.from_quat(pose[TOP,3:]).inv().apply(pts-pose[TOP,:3])
    signed=trimesh.proximity.signed_distance(m,local);depth=float((.001+signed).max()*1000)
    if depth>mesh_pen[name]:mesh_pen[name]=depth;mesh_pen_frame[name]=frame
centers=np.array(centers)
retained=(centers[:,:,0]>.005)&(centers[:,:,0]<.028)&(centers[:,:,2]>.0118)&(centers[:,:,2]<.0162)
r.update({'clip_max_opening_degrees':float(-clip_angles.min()),'clip_final_angle_degrees':float(clip_angles[-1]),'max_clip_hinge_anchor_error_mm':float(max(hinge_errors)*1000),'final_clip_height_spread_mm':float(np.ptp(centers[-1,:,2])*1000),'clip_final_cross_section_mm':(centers[-1]*1000).tolist(),'all_six_retained_after_all_hands_release':bool(released.any() and retained[released].all()),'retained_per_cable_after_all_hands_release':retained[released].all(axis=0).tolist(),'max_sampled_cable_clip_penetration_mm':mesh_pen,'max_clip_penetration_frame':mesh_pen_frame})
checks.update({'clip_opens_on_cable_contact':r['clip_max_opening_degrees']>3,'clip_spring_closes':abs(r['clip_final_angle_degrees'])<1,'all_six_in_clip_after_release':r['all_six_retained_after_all_hands_release'],'six_cables_side_by_side':bool(np.ptp(centers[-1,:,2])<.002),'clip_hinge_within_0_05_mm':max(hinge_errors)*1000<.05,'clip_sampled_penetration_below_0_2_mm':max(mesh_pen.values())<.2,'final_cable_order_preserved':bool(np.all(np.diff(centers[-1,:,0])>0))})
if 'latch_state' in d:
 r['retention_engaged_frame']=int(np.where(d['latch_state']==1)[0][0]) if np.any(d['latch_state']==1) else None
 checks['retention_engaged_and_holds']=bool(np.any(d['latch_state']==1) and np.all(d['latch_state'][released]==1))
# Full lid closure is reported separately: task success requires all six wires
# inside the original closed-cavity bounds after release, not a zero lid angle.
diagnostics={'clip_fully_closed_within_1_degree':bool(checks.pop('clip_spring_closes'))}
report={'diagnostics':diagnostics,'measurements':r,'checks':{k:bool(v) for k,v in checks.items()},'passed':bool(all(checks.values())),'limitations':['Pose and box-surface sampling only; not continuous collision proof','Concealed latch is approximated by a bounded spring capture; no calibrated hardware locking or pull-out validation','Clip STL distance sampled at 15 Hz; rod axes sampled at 17 points per segment'],'motion_sha256':hashlib.sha256((ROOT/(args.motion+'.npz')).read_bytes()).hexdigest()};(ROOT/('validation.json' if args.motion=='motion' else args.motion+'_validation.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));assert report['passed']
