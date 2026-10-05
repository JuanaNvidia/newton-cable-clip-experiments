from pathlib import Path
import os
USE_LIPS=os.environ.get("CONNECTOR_NO_LIPS")!="1"
import numpy as np
from scipy.spatial.transform import Rotation
import warp as wp
import newton
from assets import plug_parts,fixture_parts,SEAT
from robot import ARM_NAMES,GRIP_NAMES,packed,poses as robot_poses,ik,goal_flange,theta_for_gap,TOOL_ROT
ROOT=Path(__file__).resolve().parent
SOCKET=np.array([-.025,.16,.0305])
TABLE_BOXES=[([.025,.30,-.009],[.65,.70,.020])]
wp.init();wp.set_device('cuda:0')
b=newton.ModelBuilder(gravity=(0,0,-9.81))
contact=newton.ModelBuilder.ShapeConfig(density=0.,ke=1e6,kd=100.,mu=.5,gap=.0002)
visual=newton.ModelBuilder.ShapeConfig(density=0.,has_shape_collision=False,has_particle_collision=False)
def transform(p,q=None):return wp.transform(wp.vec3(*p),wp.quat_identity() if q is None else wp.quat(*q.as_quat()))
def add_parts(parts,body,offset=None,rotation=None):
 shapes=[]
 for part in parts:
  pos=np.array(part['pos']);q=Rotation.from_quat(part['q'])
  if rotation is not None:pos=rotation.apply(pos);q=rotation*q
  if offset is not None:pos+=offset
  common=dict(body=body,xform=transform(pos,q),cfg=contact if part['collision'] else visual,color=tuple(part['color']),label=part['name'])
  if part['kind']=='box':s=b.add_shape_box(**common,hx=part['size'][0]/2,hy=part['size'][1]/2,hz=part['size'][2]/2)
  else:s=b.add_shape_cylinder(**common,radius=part['size'][0],half_height=part['size'][1]/2)
  if part['collision']:shapes.append(s)
 return shapes

for center,size in TABLE_BOXES:b.add_shape_box(body=-1,hx=size[0]/2,hy=size[1]/2,hz=size[2]/2,xform=transform(center),cfg=contact)
static,moving=fixture_parts();receiver=SOCKET-SEAT
add_parts(moving,-1,receiver);add_parts(static,-1,receiver-np.array([0,-.018,.024]))
plug=b.add_link(xform=transform([.13,.16,.0053]),mass=.015,inertia=wp.mat33(*np.diag([1.2e-6,2.2e-6,3.1e-6]).reshape(-1)),label='OriginalConnector')
add_parts(plug_parts(),plug)
# Disabled bookkeeping joint avoids the runtime zero-joint shape bug.
j=b.add_joint_fixed(parent=-1,child=plug,enabled=False,collision_filter_parent=False);b.add_articulation([j])
import trimesh
robot_start=len(b.body_q);robot_names=ARM_NAMES+GRIP_NAMES
robot_ids=[];robot_shapes=[]
PICKUP_YAW=0.
open_theta=theta_for_gap(.030);hold_theta=theta_for_gap(.00599)
seed=np.array([-2.42750698,-1.55334897,2.38709924,-.83375027,-3.9983033,1.57079633])
home=np.array([.12,.25,.20]);qj=ik(goal_flange(home,open_theta,Rotation.from_euler('z',PICKUP_YAW)*TOOL_ROT),seed)
initial_robot=packed(robot_poses(qj,open_theta))
for name,q in zip(robot_names,initial_robot):
 body=b.add_link(xform=wp.transform(wp.vec3(*q[:3]),wp.quat(*q[3:])),is_kinematic=True,mass=1.,inertia=wp.mat33(*np.eye(3).reshape(-1)),label='UR5_'+name);robot_ids.append(body)
 if name in ARM_NAMES:
  from robot import URDF,origin
  link=next(x for x in URDF.findall('link') if x.get('name')==name);e=link.find('collision');path=ROOT/'robot_assets/ur'/e.find('geometry/mesh').get('filename').replace('package://','');m=trimesh.load(path);m.apply_transform(origin(e.find('origin')))
  robot_shapes.append(b.add_shape_mesh(body=body,mesh=newton.Mesh(m.vertices,m.faces.flatten()).compute_convex_hull(),cfg=contact))
 elif name.endswith('silicone_pad'):pass
 elif name.endswith('_pad'):
  padcfg=newton.ModelBuilder.ShapeConfig(density=0.,ke=1e6,kd=100.,mu=1.0,gap=.0001,margin=0.)
  for z in [.009375,.028125]:robot_shapes.append(b.add_shape_box(body=body,hx=.011,hy=.004,hz=.009375,xform=wp.transform(wp.vec3(0,-.0026,z),wp.quat_identity()),cfg=padcfg))
  from robot import fingertip_lip
  lip=fingertip_lip()
  if USE_LIPS:robot_shapes.append(b.add_shape_mesh(body=body,mesh=newton.Mesh(lip.vertices,lip.faces.flatten()).compute_convex_hull(),cfg=padcfg,label=name+'_support_lip'))
 else:
  fname=name.replace('right_','').replace('left_','');m=trimesh.load(ROOT/f'robot_assets/robotiq/robotiq_2f85/assets/{fname}.stl');m.apply_scale(.001)
  robot_shapes.append(b.add_shape_mesh(body=body,mesh=newton.Mesh(m.vertices,m.faces.flatten()).compute_convex_hull(),cfg=contact))
# Kinematic robot pairs need no contact solve. Dynamic cables retain contact
# against every robot link and both original clip components.
for i,a in enumerate(robot_shapes):
 for c in robot_shapes[i+1:]:b.shape_collision_filter_pairs.append((min(a,c),max(a,c)))
