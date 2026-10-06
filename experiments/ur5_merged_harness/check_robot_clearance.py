"""Sample recorded robot collision-mesh vertices against the table boxes.
This catches kinematic geometry passing through the bench; it is not a
continuous or complete swept-volume collision proof.
"""
from pathlib import Path
from motion_io import load_motion,motion_sha256
import hashlib
from scene import TABLE_BOXES,SOCKET_POSITIONS,SOCKET_YAWS,CLIP_ORIGINS,CLIP_YAWS
from assets import fixture_parts,SEAT
import argparse
import numpy as np,json,trimesh
from scipy.spatial.transform import Rotation as R
from robot import ARM_NAMES,GRIP_NAMES,URDF,origin,fingertip_lip
ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--motion',default='motion');args=parser.parse_args()
d=load_motion(ROOT/(args.motion+'.npz'));poses=d['poses'];offset=int(d['robot_start'])
meshes=[]
for name in ARM_NAMES:
 link=next(x for x in URDF.findall('link') if x.get('name')==name);e=link.find('collision');path=ROOT/'robot_assets/ur'/e.find('geometry/mesh').get('filename').replace('package://','');m=trimesh.load(path);m.apply_transform(origin(e.find('origin')));meshes.append(np.asarray(m.vertices))
for name in GRIP_NAMES:
 fname=name.replace('right_','').replace('left_','');m=trimesh.load(ROOT/f'robot_assets/robotiq/robotiq_2f85/assets/{fname}.stl');meshes.append(np.asarray(m.vertices)*.001)
boxes=[(np.array(c),np.array(z),R.identity(),'table') for c,z in TABLE_BOXES]
static,moving=fixture_parts()
for sk,(seat,yaw) in enumerate(zip(SOCKET_POSITIONS,SOCKET_YAWS)):
 rot=R.from_euler('z',yaw);receiver=seat-rot.apply(SEAT)
 for parts,off in [(moving,receiver),(static,receiver-rot.apply([0,-.018,.024]))]:
  for part in parts:
   if part['collision'] and part['kind']=='box':boxes.append((off+rot.apply(part['pos']),np.array(part['size']),rot*R.from_quat(part['q']),f'socket_{sk+1}'))
clipmeshes={name:trimesh.load(ROOT/('clip'+name.title()+'.stl')) for name in ['bottom','top']}
for mesh in clipmeshes.values():mesh.apply_scale(.001)
maxdepth=0.;where=None;maxima={'table':0.,'sockets':0.,'clips':0.}
tops=d['clip_top_bodies']
for frame in range(0,len(poses),6):
 obstacles=[]
 for ck,(co,cy,top) in enumerate(zip(CLIP_ORIGINS,CLIP_YAWS,tops)):
  for name,mesh in clipmeshes.items():
   cq=poses[frame,top];rr=R.from_euler('z',cy) if name=='bottom' else R.from_quat(cq[3:]);pp=co if name=='bottom' else cq[:3];corners=np.array(np.meshgrid(*zip(mesh.bounds[0],mesh.bounds[1]))).reshape(3,-1).T;world=rr.apply(corners)+pp;obstacles.append((ck,name,mesh,rr,pp,world.min(axis=0)-.0002,world.max(axis=0)+.0002))
 for i,v in enumerate(meshes):
  q=poses[frame,offset+i];pts=R.from_quat(q[3:]).apply(v)+q[:3]
  for center,size,rot,label in boxes:
   depths=(size/2-np.abs(rot.inv().apply(pts-center))).min(axis=1);depth=max(0,float(depths.max()));key='table' if label=='table' else 'sockets';maxima[key]=max(maxima[key],depth*1000)
   if depth>maxdepth:maxdepth=depth;where={'frame':frame,'link':(ARM_NAMES+GRIP_NAMES)[i],'obstacle':label}
  lo=pts.min(axis=0);hi=pts.max(axis=0)
  for ck,name,mesh,rr,pp,clo,chi in obstacles:
   if np.any(hi<clo) or np.any(lo>chi):continue
   local=rr.inv().apply(pts-pp);near=np.all((local>mesh.bounds[0]-.0002)&(local<mesh.bounds[1]+.0002),axis=1)
   if not near.any():continue
   depth=max(0.,float(trimesh.proximity.signed_distance(mesh,local[near]).max()));maxima['clips']=max(maxima['clips'],depth*1000)
   if depth>maxdepth:maxdepth=depth;where={'frame':frame,'link':(ARM_NAMES+GRIP_NAMES)[i],'obstacle':f'clip_{ck+1}_{name}'}
r={'motion_sha256':motion_sha256(ROOT/(args.motion+'.npz')),'passed':maxdepth<.0002,'max_sampled_static_penetration_mm':maxdepth*1000,'max_overlap_mm_by_obstacle':maxima,'where':where,'method':'Mesh vertices at 10 Hz against table/socket boxes and original clip meshes; not swept-volume proof'}
(ROOT/('robot_clearance_validation.json' if args.motion=='motion' else args.motion+'_robot_clearance_validation.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));assert r['passed']
