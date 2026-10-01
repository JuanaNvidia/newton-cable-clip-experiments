"""Independent pose checks for cable attachments, fixed receiver, mating, and contact."""
from pathlib import Path
import json,hashlib,argparse
import numpy as np
from scipy.spatial.transform import Rotation
from assets import RECEIVER_ORIGIN,SEAT,plug_parts,fixture_parts
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();args=p.parse_args()
d=np.load(ROOT/'motion.npz');poses=d['poses'];fps=int(d['fps']);N=int(d['segments']);ds=float(d['segment_length']);times=(np.arange(len(poses))+1)/fps
relative=[];angles=[];rot_errors=[];gaps=[];near_errors=[];far_errors=[]
for pose in poses:
 rr=Rotation.from_quat(pose[0,3:]);relative.append(rr.inv().apply(pose[1,:3]-pose[0,:3]));angles.append(rr.as_euler('xyz',degrees=True)[0]);rot_errors.append(np.degrees((rr.inv()*Rotation.from_quat(pose[1,3:])).magnitude()))
 rods=pose[2:];v=Rotation.from_quat(rods[:,3:]).apply(np.tile([0,0,ds/2],(6*N,1)));start=(rods[:,:3]-v).reshape(6,N,3);end=(rods[:,:3]+v).reshape(6,N,3);gaps.append(np.linalg.norm(end[:,:-1]-start[:,1:],axis=-1).max())
 for c in range(6):
  dx=(c-2.5)*.0025;near=pose[1,:3]+Rotation.from_quat(pose[1,3:]).apply([dx,.009,.012]);far=d['initial_cable_endpoints'][1]+[dx,0,0];near_errors.append(np.linalg.norm(start[c,0]-near));far_errors.append(np.linalg.norm(end[c,-1]-far))
relative=np.array(relative);released=times>=5.3
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
r={'finite':bool(np.isfinite(poses).all()),'plug_start_degrees':float(rot_errors[0]),'plug_angle_at_2_5s_degrees':float(rot_errors[min(149,len(rot_errors)-1)]),'receiver_start_degrees':float(angles[0]),'receiver_final_degrees':float(angles[-1]),'receiver_angle_at_1s_degrees':float(angles[min(59,len(angles)-1)]),'receiver_rotation_degrees':float(max(angles)-min(angles)),'final_plug_in_receiver_mm':(relative[-1]*1000).tolist(),'ideal_seat_mm':(SEAT*1000).tolist(),'final_relative_angle_degrees':float(rot_errors[-1]),'max_after_release_seat_error_mm':float(np.linalg.norm(relative[released]-SEAT,axis=1).max()*1000) if released.any() else None,'max_after_release_relative_angle_degrees':float(np.array(rot_errors)[released].max()) if released.any() else None,'max_receiver_translation_error_mm':float(np.linalg.norm(poses[:,0,:3]-RECEIVER_ORIGIN,axis=1).max()*1000),'max_cable_joint_gap_mm':float(max(gaps)*1000),'max_near_attachment_error_mm':float(max(near_errors)*1000),'max_far_attachment_error_mm':float(max(far_errors)*1000),'max_sampled_plug_receiver_penetration_mm':penetration*1000,'release_observation_seconds':float(max(0,times[-1]-5.2))}
checks={'finite':r['finite'],'receiver_stays_fixed':bool(np.max(np.abs(poses[:,0,:]-np.array([*RECEIVER_ORIGIN,0,0,0,1])))<1e-7),'plug_initially_angled':rot_errors[0]>20,'plug_still_angled_at_engagement':rot_errors[min(149,len(rot_errors)-1)]>15,'plug_rotates_down':rot_errors[-1]<2,'seated_after_hand_release':released.any() and r['max_after_release_seat_error_mm']<1,'aligned_after_hand_release':released.any() and r['max_after_release_relative_angle_degrees']<2,'fixed_receiver_position_within_0_1_mm':r['max_receiver_translation_error_mm']<.1,'rod_joints_within_0_1_mm':r['max_cable_joint_gap_mm']<.1,'cables_attached':max(r['max_near_attachment_error_mm'],r['max_far_attachment_error_mm'])<.1,'sampled_penetration_below_0_2_mm':r['max_sampled_plug_receiver_penetration_mm']<.2}
report={'measurements':r,'checks':{k:bool(v) for k,v in checks.items()},'passed':bool(all(checks.values())),'limitations':['Pose and box-surface sampling only; not continuous collision proof','Concealed latch is not modeled; no positive locking or pull-out validation'],'motion_sha256':hashlib.sha256((ROOT/'motion.npz').read_bytes()).hexdigest()};(ROOT/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));assert report['passed']
