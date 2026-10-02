"""Sample original clip STL distances against recorded cable capsule axes."""
from pathlib import Path
import hashlib
import argparse
import numpy as np,json
from scipy.spatial.transform import Rotation as R
import trimesh
from clip_geometry import CLIP_ORIGIN
ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--motion',default='motion');args=parser.parse_args()
d=np.load(ROOT/(args.motion+'.npz'));poses=d['poses'];ids=d['rod_ids'];ds=float(d['segment_length']);top=int(d['clip_top_body'])
meshes={k:trimesh.load(ROOT/('clip'+k.title()+'.stl')) for k in ['top','bottom']}
for m in meshes.values():m.apply_scale(.001)
maximum={'top':0.,'bottom':0.};frames={'top':None,'bottom':None}
for frame in range(0,len(poses),4):
 q=poses[frame];rods=q[ids];v=R.from_quat(rods[:,3:]).apply(np.tile([0,0,1],(len(ids),1)))
 pts=(rods[:,:3,None]+v[:,:,None]*np.linspace(-ds/2,ds/2,17)).transpose(0,2,1).reshape(-1,3);local=pts-CLIP_ORIGIN;near=(local[:,0]>-.055)&(local[:,0]<.056)&(local[:,1]>-.035)&(local[:,1]<.005)&(local[:,2]<.055);pts=pts[near]
 if not len(pts):continue
 for name,m in meshes.items():
  local=pts-CLIP_ORIGIN if name=='bottom' else R.from_quat(q[top,3:]).inv().apply(pts-q[top,:3]);depth=max(0,float((.001+trimesh.proximity.signed_distance(m,local)).max()*1000))
  if depth>maximum[name]:maximum[name]=depth;frames[name]=frame
r={'motion_sha256':hashlib.sha256((ROOT/(args.motion+'.npz')).read_bytes()).hexdigest(),'max_sampled_capsule_clip_penetration_mm':maximum,'frames':frames,'passed':max(maximum.values())<.2,'method':'Original STL signed distance, 15 Hz and 17 axis samples per capsule; not continuous collision proof'}
(ROOT/('contact_validation.json' if args.motion=='motion' else args.motion+'_contact_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));assert r['passed']
