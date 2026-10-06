"""Sample cable penetration into pad boxes; no fingertip wedges."""
from pathlib import Path
from motion_io import load_motion,motion_sha256
import hashlib
import argparse,json
import numpy as np,trimesh
from scipy.spatial.transform import Rotation as R
from robot import fingertip_lip
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--motion',default='motion');a=p.parse_args()
d=load_motion(ROOT/(a.motion+'.npz'));poses=d['poses'];ids=d['rod_ids'];ds=float(d['segment_length']);names=list(d['robot_names']);base=int(d['robot_start']);lip=fingertip_lip()
maxima={'pads':0.,'support_lips':0.};where={k:None for k in maxima}
for frame in range(0,len(poses),4):
 q=poses[frame];rods=q[ids];directions=R.from_quat(rods[:,3:]).apply(np.tile([0,0,1],(len(ids),1)))
 pts=(rods[:,:3,None]+directions[:,:,None]*np.linspace(-ds/2,ds/2,17)).transpose(0,2,1).reshape(-1,3)
 for side in ['left','right']:
  pad=q[base+names.index(side+'_pad')];local=R.from_quat(pad[3:]).inv().apply(pts-pad[:3]);near=(np.abs(local[:,0])<.014)&(local[:,1]>-.013)&(local[:,1]<.004)&(local[:,2]>-.004)&(local[:,2]<.042);local=local[near]
  if not len(local):continue
  delta=np.abs(local-[0,-.0026,.01875])-[.011,.004,.01875];outside=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(axis=-1),0)
  values={'pads':max(0,float((.001-outside).max()*1000)),'support_lips':0.}
  for key,value in values.items():
   if value>maxima[key]:maxima[key]=value;where[key]={'frame':frame,'side':side}
report={'motion_sha256':motion_sha256(ROOT/(a.motion+'.npz')),'passed':max(maxima.values())<.2,'max_sampled_cable_gripper_penetration_mm':maxima,'where':where,'method':'Pad box signed distances; support lips absent, 15 Hz, 17 axis points per capsule; not continuous collision proof'}
(ROOT/('gripper_contact_validation.json' if a.motion=='motion' else a.motion+'_gripper_contact_validation.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));assert report['passed']
