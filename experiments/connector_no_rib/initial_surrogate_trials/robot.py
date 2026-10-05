"""UR5 CB3 URDF kinematics and Robotiq 2F-85 closed-loop kinematics.
Position-controlled poses only; no additional physics engine is used.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares,root,brentq
ROOT=Path(__file__).resolve().parent
BASE=np.array([.50,.52,.001])
def T(p=(0,0,0),r=None):
 m=np.eye(4);m[:3,3]=p
 if r is not None:m[:3,:3]=r.as_matrix()
 return m
def rx(a):return T(r=R.from_rotvec([a,0,0]))
def origin(e):
 if e is None:return np.eye(4)
 return T(np.fromstring(e.get('xyz','0 0 0'),sep=' '),R.from_euler('xyz',np.fromstring(e.get('rpy','0 0 0'),sep=' ')))
URDF=ET.parse(ROOT/'robot_assets/ur5.urdf').getroot()
JOINTS=list(URDF.findall('joint'));LINKS=list(URDF.findall('link'))
ARM_NAMES=[e.get('name') for e in LINKS if e.find('visual') is not None]
def fk(q):
 out={'base_link':T(BASE)};i=0
 for j in JOINTS:
  name=j.find('child').get('link');parent=j.find('parent').get('link');m=origin(j.find('origin'))
  if j.get('type')!='fixed':
   axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');m=m@T(r=R.from_rotvec(q[i]*axis));i+=1
  out[name]=out[parent]@m
 return out
# Top-down grasp: pad width Y, closing X, approach -Z.
# TCP is 1.05 mm above the pad tips, allowing tabletop clearance.
TOOL_ROT=R.from_matrix(np.array([[0,1,0],[1,0,0],[0,0,-1]]))
MOUNT=T([0,0,.0108],R.from_euler('z',-90,degrees=True))
def gripper_local(theta):
 D=np.array([0,.0306011,.054904]);S=np.array([0,.0132,.0609]);C=np.array([0,.0315,-.0041]);F=np.array([0,.055,.0375]);pivot=np.array([0,-.018,.0065]);attach=S+F-D-C
 def loop(x):
  s,c=x;fm=T(S)@rx(s)@T(F)@T(pivot)@rx(-s)@T(-pivot);cm=T(D)@rx(theta)@T(C)@rx(c)
  return (fm[:3,3]-(cm@np.r_[attach,1])[:3])[1:]
 s,c=root(loop,[theta,-theta]).x
 assert np.linalg.norm(loop([s,c]))<1e-8
 out={'base_mount':np.linalg.inv(MOUNT),'base':np.eye(4)}
 for side,mirror in [('right',np.eye(4)),('left',T(r=R.from_euler('z',180,degrees=True)))]:
  out[side+'_driver']=mirror@T(D)@rx(theta)
  out[side+'_coupler']=out[side+'_driver']@T(C)@rx(c)
  out[side+'_spring_link']=mirror@T(S)@rx(s)
  out[side+'_follower']=out[side+'_spring_link']@T(F)@T(pivot)@rx(-s)@T(-pivot)
  out[side+'_pad']=out[side+'_follower']@T([0,-.0189,.01352])
  out[side+'_silicone_pad']=out[side+'_pad'].copy()
 return out
GRIP_NAMES=list(gripper_local(0))
def gap(theta):
 m=gripper_local(theta)['right_pad'];return 2*(m@np.array([0,-.0066,.01875,1]))[1]
def theta_for_gap(width):return brentq(lambda a:gap(a)-width,0,.8)
def pinch_z(theta):return (gripper_local(theta)['right_pad']@np.array([0,-.0066,.01875,1]))[2]
def goal_flange(center,theta,tool_rotation=None):
 rotation=TOOL_ROT if tool_rotation is None else tool_rotation
 base=T(center-rotation.apply([0,0,pinch_z(theta)+.01770]),rotation)
 return base@np.linalg.inv(MOUNT)
def ik(target,seed):
 def error(q):
  m=fk(q)['tool0'];return np.r_[(m[:3,3]-target[:3,3])*3,R.from_matrix(target[:3,:3]@m[:3,:3].T).as_rotvec()]
 res=least_squares(error,seed,bounds=(-2*np.pi,2*np.pi),max_nfev=80,ftol=1e-9,xtol=1e-9,gtol=1e-9)
 assert np.linalg.norm(error(res.x))<1e-5,(res.x,error(res.x))
 return res.x

def poses(q,theta):
 arm=fk(q);base=arm['tool0']@MOUNT;return [arm[n] for n in ARM_NAMES]+[base@m for m in gripper_local(theta).values()]
def packed(ms):return np.array([np.r_[m[:3,3],R.from_matrix(m[:3,:3]).as_quat()] for m in ms],np.float32)
def fingertip_lip():
 """Thin wedge support insert, in the original pad frame (metres).
 Leading edge is 0.10 mm thick; back is 1.0 mm, protrusion 2.5 mm.
 """
 import trimesh
 yz=[[-.0066,.0375],[-.0091,.0375],[-.0091,.0374],[-.0066,.0365]]
 points=np.array([[x,y,z] for x in [-.011,.011] for y,z in yz])
 return trimesh.convex.convex_hull(points)

def visual_meshes():
 """Flatten authored DAE nodes and material colors into per-link mesh pieces."""
 import trimesh
 result=[]
 for name in ARM_NAMES:
  link=next(x for x in LINKS if x.get('name')==name);v=link.find('visual');path=ROOT/'robot_assets/ur'/v.find('geometry/mesh').get('filename').replace('package://','')
  scene=trimesh.load(path,force='scene');pieces=[]
  for node in scene.graph.nodes_geometry:
   m,g=scene.graph[node];mesh=scene.geometry[g].copy();mesh.apply_transform(origin(v.find('origin'))@m)
   color=np.array([.68,.70,.72])
   if hasattr(mesh.visual,'material'):
    mat=mesh.visual.material
    if hasattr(mat,'diffuse'):color=np.array(mat.diffuse[:3])/255
    elif hasattr(mat,'baseColorFactor'):color=np.array(mat.baseColorFactor[:3])/255
   pieces.append((mesh,color))
  result.append(pieces)
 for name in GRIP_NAMES:
  fname=name.replace('right_','').replace('left_','');mesh=trimesh.load(ROOT/f'robot_assets/robotiq/robotiq_2f85/assets/{fname}.stl');mesh.apply_scale(.001)
  color=(.55,.57,.59) if fname=='base_mount' else (.46,.46,.46) if fname=='driver' else (.15,.16,.17)
  pieces=[(mesh,color)]
  if name.endswith('_pad') and not name.endswith('silicone_pad'):pieces.append((fingertip_lip(),(.72,.48,.10)))
  result.append(pieces)
 return result
if __name__=='__main__':
 print('GAPS',[(a,gap(a),pinch_z(a)) for a in [0,.4,.7,.8]])
 t=theta_for_gap(.0018);target=goal_flange(np.array([.037,.255,.026]),t)
 for seed in [[0,-1.5,1.8,-1.8,-1.57,0],[3,-1.5,-1.8,-1.3,1.57,0]]:
  try:q=ik(target,np.array(seed));print('IK',q);break
  except AssertionError:pass
