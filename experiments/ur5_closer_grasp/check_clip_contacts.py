"""Sample all three original clip meshes against solved capsule axes."""
from pathlib import Path
from motion_io import load_motion,motion_sha256
import argparse,json,hashlib
import numpy as np,trimesh
from scipy.spatial.transform import Rotation as R
from scene import CLIP_ORIGINS,CLIP_YAWS
root=Path(__file__).resolve().parent;p=argparse.ArgumentParser();p.add_argument('--motion',default='motion');a=p.parse_args();d=load_motion(root/(a.motion+'.npz'));q=d['poses'];ids=d['rod_ids'];ds=float(d['segment_length']);tops=d['clip_top_bodies']
meshes={k:trimesh.load(root/('clip'+k.title()+'.stl')) for k in ['top','bottom']}
for m in meshes.values():m.apply_scale(.001)
maximum={f'clip{k+1}_{n}':0. for k in range(3) for n in meshes};where={}
for frame in range(0,len(q),4):
 if frame%240==0:print(f'Original-mesh audit: frame {frame}/{len(q)-1}',flush=True)
 bodies=q[frame];rods=bodies[ids];v=R.from_quat(rods[:,3:]).apply(np.tile([0,0,1],(len(ids),1)));pts=(rods[:,:3,None]+v[:,:,None]*np.linspace(-ds/2,ds/2,17)).transpose(0,2,1).reshape(-1,3)
 for k,(origin,yaw,top) in enumerate(zip(CLIP_ORIGINS,CLIP_YAWS,tops)):
  for name,m in meshes.items():
   local=R.from_euler('z',yaw).inv().apply(pts-origin) if name=='bottom' else R.from_quat(bodies[top,3:]).inv().apply(pts-bodies[top,:3])
   near=np.all((local>=m.bounds[0]-.001)&(local<=m.bounds[1]+.001),axis=1)
   if not near.any():continue
   depth=max(0,float((.001+trimesh.proximity.signed_distance(m,local[near])).max()*1000));key=f'clip{k+1}_{name}'
   if depth>maximum[key]:maximum[key]=depth;where[key]=frame
r=dict(motion_sha256=motion_sha256(root/(a.motion+'.npz')),passed=max(maximum.values())<.2,max_sampled_capsule_clip_penetration_mm=maximum,frames=where,method='Original STL signed distances with per-mesh local bounds including the rotated lid, 15 Hz and 17 samples per capsule; not continuous collision proof')
(root/('contact_validation.json' if a.motion=='motion' else a.motion+'_contact_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
