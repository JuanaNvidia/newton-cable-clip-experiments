"""Sample recorded robot collision-mesh vertices against the table boxes.
This catches kinematic geometry passing through the bench; it is not a
continuous or complete swept-volume collision proof.
"""
from pathlib import Path
from motion_io import load_motion,motion_sha256
import hashlib
from scene import TABLE_BOXES
import argparse
import numpy as np,json,trimesh
from scipy.spatial.transform import Rotation as R
from robot import ARM_NAMES,GRIP_NAMES,URDF,origin,fingertip_lip
ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--motion',default='motion');args=parser.parse_args()
d=load_motion(ROOT/(args.motion+'.npz'));poses=d['poses'];offset=int(d['robot_start']);with_lips=json.loads((ROOT/(args.motion+'.json')).read_text()).get('existing_fingertip_support_lips',True)
meshes=[]
for name in ARM_NAMES:
 link=next(x for x in URDF.findall('link') if x.get('name')==name);e=link.find('collision');path=ROOT/'robot_assets/ur'/e.find('geometry/mesh').get('filename').replace('package://','');m=trimesh.load(path);m.apply_transform(origin(e.find('origin')));meshes.append(np.asarray(m.vertices))
for name in GRIP_NAMES:
 fname=name.replace('right_','').replace('left_','');m=trimesh.load(ROOT/f'robot_assets/robotiq/robotiq_2f85/assets/{fname}.stl');meshes.append(np.vstack([np.asarray(m.vertices)*.001,np.asarray(fingertip_lip().vertices)]) if with_lips and name.endswith('_pad') and not name.endswith('silicone_pad') else np.asarray(m.vertices)*.001)
from assets import fixture_parts,SEAT
boxes=list(TABLE_BOXES)
static,moving=fixture_parts();receiver=np.array([-.025,.16,.0305])-SEAT
for parts,off in [(moving,receiver),(static,receiver-np.array([0,-.018,.024]))]:
 for part in parts:
  if part['collision'] and part['kind']=='box':boxes.append((np.array(part['pos'])+off,part['size']))
maxdepth=0.;where=None;by_phase={}
for frame in range(0,len(poses),6):
 for i,v in enumerate(meshes):
  q=poses[frame,offset+i];pts=R.from_quat(q[3:]).apply(v)+q[:3]
  for center,size in boxes:
   depths=(np.array(size)/2-np.abs(pts-center)).min(axis=1);depth=max(0,float(depths.max()))
   phase=str(d['phases'][frame]);by_phase[phase]=max(by_phase.get(phase,0.),depth*1000)
   if depth>maxdepth:maxdepth=depth;where={'frame':frame,'link':(ARM_NAMES+GRIP_NAMES)[i]}
r={'max_overlap_mm_by_phase':by_phase,'motion_sha256':motion_sha256(ROOT/(args.motion+'.npz')),'passed':maxdepth<.0002,'max_sampled_static_penetration_mm':maxdepth*1000,'where':where,'method':'Mesh vertices at 10 Hz against table and socket boxes; not swept-volume proof'}
(ROOT/('robot_clearance_validation.json' if args.motion=='motion' else args.motion+'_robot_clearance_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));assert r['passed']
