"""Sample recorded robot collision-mesh vertices against the table boxes.
This catches kinematic geometry passing through the bench; it is not a
continuous or complete swept-volume collision proof.
"""
from pathlib import Path
import argparse
import numpy as np,json,trimesh
from scipy.spatial.transform import Rotation as R
from robot import ARM_NAMES,GRIP_NAMES,URDF,origin
ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--motion',default='motion');args=parser.parse_args()
d=np.load(ROOT/(args.motion+'.npz'));poses=d['poses'];offset=int(d['robot_start'])
meshes=[]
for name in ARM_NAMES:
 link=next(x for x in URDF.findall('link') if x.get('name')==name);e=link.find('collision');path=ROOT/'robot_assets/ur'/e.find('geometry/mesh').get('filename').replace('package://','');m=trimesh.load(path);m.apply_transform(origin(e.find('origin')));meshes.append(np.asarray(m.vertices))
for name in GRIP_NAMES:
 fname=name.replace('right_','').replace('left_','');m=trimesh.load(ROOT/f'robot_assets/robotiq/robotiq_2f85/assets/{fname}.stl');meshes.append(np.asarray(m.vertices)*.001)
boxes=[([-.12,.0475,-.010],[.38,.335,.016]),([-.173,.0475,-.0005],[.274,.335,.003]),([.053,.0475,-.0005],[.034,.335,.003]),([0,-.078,-.0005],[.072,.084,.003]),([0,.1255,-.0005],[.072,.179,.003])]
maxdepth=0.;where=None
for frame in range(0,len(poses),6):
 for i,v in enumerate(meshes):
  q=poses[frame,offset+i];pts=R.from_quat(q[3:]).apply(v)+q[:3]
  for center,size in boxes:
   depths=(np.array(size)/2-np.abs(pts-center)).min(axis=1);depth=max(0,float(depths.max()))
   if depth>maxdepth:maxdepth=depth;where={'frame':frame,'link':(ARM_NAMES+GRIP_NAMES)[i]}
r={'passed':maxdepth<.0002,'max_sampled_table_penetration_mm':maxdepth*1000,'where':where,'method':'Mesh vertices at 10 Hz against solid table boxes; not swept-volume proof'}
(ROOT/('robot_clearance_validation.json' if args.motion=='motion' else args.motion+'_robot_clearance_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));assert r['passed']
