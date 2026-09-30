"""Independently check endpoint travel and rod continuity in recorded poses."""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
d=np.load(ROOT/'arch_motion.npz');q=d['poses'];n=int(d['segments']);h=float(d['segment_length'])/2;s=d['spans'];gaps=[];errors=[];separations=[]
for c in range(3):
 rods=q[:,c*n:(c+1)*n];direction=Rotation.from_quat(rods[:,:,3:].reshape(-1,4)).apply(np.tile([0,0,h],(len(q)*n,1))).reshape(len(q),n,3)
 start=rods[:,:,:3]-direction;end=rods[:,:,:3]+direction
 gaps.append(float(np.linalg.norm(end[:,:-1]-start[:,1:],axis=-1).max()))
 for side,points in [(-1,start[:,0]),(1,end[:,-1])]:
  target=np.column_stack([np.full(len(q),(c-1)*.045),side*s/2,np.full(len(q),.015)])
  errors.append(float(np.linalg.norm(points-target,axis=-1).max()))
 separations.append(float(np.linalg.norm(end[-1,-1]-start[-1,0])))
checks={'finite':bool(np.isfinite(q).all()),'final_command_76_2_mm':abs(float(s[-1])-.0762)<1e-7,'actual_endpoint_span_within_0_05_mm':all(abs(x-.0762)<.00005 for x in separations),'anchors_within_0_05_mm':max(errors)<.00005,'joints_within_0_05_mm':max(gaps)<.00005}
r={'checks':checks,'passed':all(checks.values()),'motion_sha256':hashlib.sha256((ROOT/'arch_motion.npz').read_bytes()).hexdigest()}
(ROOT/'recomputed_validation.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));assert r['passed']
