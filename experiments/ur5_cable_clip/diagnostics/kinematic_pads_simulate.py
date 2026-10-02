"""UR5/Robotiq contact-only pickup and insertion of six Newton cables."""
from pathlib import Path
import argparse,json,math,time
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import warp as wp
import newton
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,HOLE_TOP,clip_meshes
from robot import ARM_NAMES,GRIP_NAMES,poses as robot_poses,packed,ik,goal_flange,theta_for_gap,gripper_local
from assets import RECEIVER_ORIGIN,OPEN_DEG,SEAT,plug_parts,fixture_parts
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=16);p.add_argument('--output',default='motion');p.add_argument('--no-clip-contact',action='store_true');args=p.parse_args()
wp.init();wp.set_device('cuda:0')
CLIP_SPRING_MULTIPLIER=10.
CLIP_PRELOAD=.03
N=91;COUNT=6;L=.4572;DS=L/N;R=.001;PITCH=.0025;EI=.0011459155902616466
rot0=Rotation.identity();initial=RECEIVER_ORIGIN+SEAT
far=np.array([0,.402,.010]);farrot=Rotation.from_euler('z',180,degrees=True)
b=newton.ModelBuilder(gravity=(0.,0.,-9.81))
contact=newton.ModelBuilder.ShapeConfig(density=0.,ke=1e6,kd=100.,mu=.5,gap=.0002)
visual=newton.ModelBuilder.ShapeConfig(density=0.,has_shape_collision=False,has_particle_collision=False)
def add_parts(parts,body,offset=None,rotation=None):
 shapes=[]
 for part in parts:
  pos=np.array(part['pos']);q=Rotation.from_quat(part['q'])
  if rotation is not None:pos=rotation.apply(pos);q=rotation*q
  if offset is not None:pos+=offset
  common=dict(body=body,xform=wp.transform(wp.vec3(*pos),wp.quat(*q.as_quat())),cfg=contact if part['collision'] else visual,color=tuple(part['color']),label=part['name'])
  if part['kind']=='box':s=b.add_shape_box(**common,hx=part['size'][0]/2,hy=part['size'][1]/2,hz=part['size'][2]/2)
  else:s=b.add_shape_cylinder(**common,radius=part['size'][0],half_height=part['size'][1]/2)
  if part['collision']:shapes.append(s)
 return shapes
# Wood below a metal sheet with a genuine rectangular opening around the fixture.
b.add_shape_box(body=-1,hx=.19,hy=.1675,hz=.008,xform=wp.transform(wp.vec3(-.12,.0475,-.010),wp.quat_identity()),cfg=contact)
for center,extent in [((-.173, .0475,-.0005),(.274,.335,.003)),((.053,.0475,-.0005),(.034,.335,.003)),((0,-.078,-.0005),(.072,.084,.003)),((0,.1255,-.0005),(.072,.179,.003))]:
 b.add_shape_box(body=-1,hx=extent[0]/2,hy=extent[1]/2,hz=extent[2]/2,xform=wp.transform(wp.vec3(*center),wp.quat_identity()),cfg=contact)
static,moving=fixture_parts();add_parts(static,-1)
receiver=b.add_link(xform=wp.transform(wp.vec3(*RECEIVER_ORIGIN),wp.quat_identity()),is_kinematic=True,mass=.025,com=wp.vec3(0,.018,0),inertia=wp.mat33(*np.diag(np.array([.040**2+.010**2,.050**2+.010**2,.050**2+.040**2])*.025/12).reshape(-1)),label='FixedReceiver')
receiver_shapes=add_parts(moving,receiver)
plug=b.add_link(is_kinematic=True,xform=wp.transform(wp.vec3(*initial),wp.quat(*rot0.as_quat())),mass=.015,inertia=wp.mat33(*np.diag(np.array([.027**2+.008**2,.042**2+.008**2,.042**2+.027**2])*.015/12).reshape(-1)),label='CableEndPlug')
plug_shapes=add_parts(plug_parts(),plug)

# Straight material rods in a bent initial configuration, connector already locked.
# A free tail extends beyond the table edge; no distal anchor or virtual hands.
a0=initial+np.array([0,.009,.012])
u=np.linspace(0,1,10001);yy=.44*u
xx=.20*yy
zz=.0425+.15*yy-.09*(yy/.44)**2
curve=np.column_stack([xx,.009+yy,zz]);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(curve,axis=0),axis=1))]
points=np.column_stack([np.interp(np.linspace(0,L,N+1),arc,curve[:,k]) for k in range(3)])
a1=points[-1]
cfg=newton.ModelBuilder.ShapeConfig(density=1000*DS/(DS+4*R/3),ke=1e6,kd=100.,mu=.5,gap=.0002)
rod_ids=[];attachment_points=[];initial_quats=[]
for c in range(COUNT):
 dx=(c-2.5)*PITCH;chain=[];joints=[];rots=[]
 for i in range(N):
  v=(points[i+1]-points[i])/DS;q=Rotation.align_vectors([v],[[0,0,1]])[0];pos=(points[i]+points[i+1])/2+np.array([dx,0,0]);body=b.add_link(xform=wp.transform(wp.vec3(*pos),wp.quat(*q.as_quat())),label=f'Cable_{c}_{i:02}');chain.append(body);rots.append(q)
  b.add_shape_capsule(body=body,radius=R,half_height=DS/2,cfg=cfg)
  if i:joints.append(b.add_joint_rod(parent=chain[-2],child=body,parent_xform=wp.transform(wp.vec3(0,0,DS/2),wp.quat_identity()),child_xform=wp.transform(wp.vec3(0,0,-DS/2),wp.quat_identity()),stretch_stiffness=1e6,stretch_damping=.1,bend_stiffness=EI/DS,bend_damping=.005729577951308233*.005/DS))
 b.add_articulation(joints)
 parentq=(rot0.inv()*rots[0]).as_quat();j=b.add_joint_fixed(parent=plug,child=chain[0],parent_xform=wp.transform(wp.vec3(dx,.009,.012),wp.quat(*parentq)),child_xform=wp.transform(wp.vec3(0,0,-DS/2),wp.quat_identity()));b.add_articulation([j])
 rod_ids+=chain
# Reuse original, unscaled clip and its convex lid decomposition.
meshset=clip_meshes()
def clip_shape(data,body,cfg,offset=None,convex=False):
 v,f=data;m=newton.Mesh(v,f)
 if convex:m=m.compute_convex_hull()
 kw={} if offset is None else {'xform':wp.transform(wp.vec3(*offset),wp.quat_identity())}
 return b.add_shape_mesh(body=body,mesh=m,cfg=cfg,**kw)
clip_shape(meshset['bottom'],-1,contact,CLIP_ORIGIN)
clip_shape(meshset['support'],-1,contact,CLIP_ORIGIN,True)
top=b.add_link(xform=wp.transform(wp.vec3(*(CLIP_ORIGIN+HOLE_BOTTOM-HOLE_TOP)),wp.quat_identity()),label='SpringClipTop')
clipcfg=newton.ModelBuilder.ShapeConfig(density=1000.,ke=1e6,kd=100.,mu=.5,gap=.0002,margin=0.)
top_shapes=[clip_shape(m,top,clipcfg,convex=True) for m in meshset['convex_top']]
hinge=b.add_joint_revolute(parent=-1,child=top,parent_xform=wp.transform(wp.vec3(*(CLIP_ORIGIN+HOLE_BOTTOM)),wp.quat_identity()),child_xform=wp.transform(wp.vec3(*HOLE_TOP),wp.quat_identity()),axis=wp.vec3(0.,1.,0.),target_pos=0.,target_ke=CLIP_SPRING_MULTIPLIER*.0002*180/math.pi,target_kd=CLIP_SPRING_MULTIPLIER*.000005*180/math.pi,limit_lower=math.radians(-75),limit_upper=0.,collision_filter_parent=False,label='OriginalSpringClipHinge')
b.add_articulation([hinge])
if args.no_clip_contact:
 for a in [shape for body in rod_ids for shape in b.body_shapes[body]]:
  for c in top_shapes:b.shape_collision_filter_pairs.append((min(a,c),max(a,c)))
# Robot bodies are driven by URDF FK. Finger poses obey the four-bar loop.
# All cable forces are gravity, rod elasticity and Newton collision contacts.
import trimesh
robot_start=len(b.body_q);robot_names=ARM_NAMES+GRIP_NAMES
robot_ids=[];robot_shapes=[]
open_theta=theta_for_gap(.060);hold_theta=theta_for_gap(.00199)
seed=np.array([-2.42750698,-1.55334897,2.38709924,-.83375027,-3.9983033,1.57079633])
home=np.array([.13,.26,.13]);qj=ik(goal_flange(home,open_theta),seed)
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
 else:
  fname=name.replace('right_','').replace('left_','');m=trimesh.load(ROOT/f'robot_assets/robotiq/robotiq_2f85/assets/{fname}.stl');m.apply_scale(.001)
  robot_shapes.append(b.add_shape_mesh(body=body,mesh=newton.Mesh(m.vertices,m.faces.flatten()).compute_convex_hull(),cfg=contact))
# Kinematic robot pairs need no contact solve. Dynamic cables retain contact
# against every robot link and both original clip components.
for i,a in enumerate(robot_shapes):
 for c in robot_shapes[i+1:]:b.shape_collision_filter_pairs.append((min(a,c),max(a,c)))
b.color(balance_colors=False);model=b.finalize(device='cuda:0')
I=np.array(b.body_inertia).reshape(-1,3,3);assert np.linalg.eigvalsh(I).min()>0
model.body_inertia.assign(I.astype(np.float32));inv_I=np.linalg.inv(I).astype(np.float32);inv_I[[receiver,plug]+robot_ids]=0;model.body_inv_inertia.assign(inv_I)
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=100000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=60,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=512,friction_epsilon=1e-4,rigid_joint_linear_ke=1e7,rigid_joint_angular_ke=1e5)
solver.joint_rod_rest_kb_local.zero_();solver.joint_rod_rest_twist.zero_()
s0=model.state();s1=model.state();ctrl=model.control();contacts=pipeline.contacts();dt=1/3840
robot_prev=wp.array(initial_robot,dtype=wp.transform);robot_next=wp.array(initial_robot,dtype=wp.transform)
substep=wp.zeros(1,dtype=wp.int32)
@wp.kernel
def drive(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),previous:wp.array(dtype=wp.transform),next:wp.array(dtype=wp.transform),counter:wp.array(dtype=wp.int32),offset:int):
 j=wp.tid();u=float(counter[0]+1)/64.;a=previous[j];bq=next[j];pa=wp.transform_get_translation(a);pb=wp.transform_get_translation(bq);ra=wp.transform_get_rotation(a);rb=wp.transform_get_rotation(bq)
 q[offset+j]=wp.transform(pa+(pb-pa)*u,wp.quat_slerp(ra,rb,u))
 e=rb*wp.quat_inverse(ra)
 if e[3]<0.:e=-e
 qd[offset+j]=wp.spatial_vector((pb-pa)*60.,wp.vec3(e[0],e[1],e[2])*120.)
@wp.kernel
def preload(force:wp.array(dtype=wp.spatial_vector),body:int):force[body]=force[body]+wp.spatial_vector(wp.vec3(0.),wp.vec3(0.,.03,0.))
@wp.kernel
def tick(counter:wp.array(dtype=wp.int32)):counter[0]+=1
with wp.ScopedCapture() as cap:
 for _ in range(64):
  s0.clear_forces();wp.launch(preload,1,inputs=[s0.body_f,top]);wp.launch(drive,len(robot_ids),inputs=[s0.body_q,s0.body_qd,robot_prev,robot_next,substep,robot_start]);pipeline.collide(s0,contacts);solver.step(s0,s1,ctrl,contacts,dt);wp.launch(tick,1,inputs=[substep]);s0,s1=s1,s0
poses=[];counts=[];jhistory=[];widths=[];targets=[];start=time.time();grasp=None;lastrobot=initial_robot;waypoints=None;grasp_segments=np.full(6,49,dtype=int)
def cable_center(q):
 # Follow the same material location, avoiding ambiguous intersections when
 # the free tails swing back through the pickup plane.
 chains=q[np.array(rod_ids).reshape(COUNT,N),:3]
 point=chains[:,gi,:].mean(axis=0);point[1]=max(.230,point[1]);return point

for frame in range(round(args.seconds*60)):
 t=frame/60
 if frame==90:
  stateq=s0.body_q.numpy();chains=stateq[np.array(rod_ids).reshape(COUNT,N),:3]
  gi=int(np.argmin(abs(chains[:,:,1].mean(axis=0)-.244)));grasp=chains[:,gi,:].mean(axis=0)
  # The downstream grasp pinches the bundle near the bottom of the stock pads.
  print('GRASP',grasp,'spread',np.ptp(chains[:,gi,:],axis=0),'segment',gi,flush=True)
  above=grasp+np.array([.075,0,0]);outside=np.array([grasp[0],grasp[1],.026]);through=np.array([-.008,grasp[1]+.017,.026]);inside=through-np.array([0,0,.014]);clear=np.array([.13,inside[1],inside[2]])
  waypoints=[(1.5,home,.060),(2.8,above,.060),(3.8,grasp,.060),(4.5,grasp,.00199),(5.3,grasp+np.array([0,0,.014]),.00199),(6.3,outside,.00199),(9.0,through,.00199),(9.8,inside,.00199),(10.4,inside,.00199),(11.0,inside,.060),(12.2,clear,.060),(13.4,home,.060),(16,home,.060)]
 center=home.copy();width=.060
 if waypoints:
  for a,z in zip(waypoints[:-1],waypoints[1:]):
   u=float(np.clip((t-a[0])/(z[0]-a[0]),0,1));u=u*u*(3-2*u);center=(1-u)*a[1]+u*z[1];width=(1-u)*a[2]+u*z[2]
   if t<=z[0]:break
 # Scripted state feedback during approach follows the still-swinging
 # bundle at a fixed Y plane. It commands only robot poses, never cable forces.
 if 1.5<=t<=3.8:
  current=s0.body_q.numpy();tracked=cable_center(current)
  if t<2.8:
   u=np.clip((t-1.5)/1.3,0,1);u=u*u*(3-2*u);center=home*(1-u)+(tracked+np.array([.075,0,0]))*u
  elif t<3.8:
   u=np.clip((t-2.8),0,1);u=u*u*(3-2*u);center=tracked+np.array([.075*(1-u),0,0])
  else:center=tracked
  if frame==228:
   grasp=center.copy();chains=current[np.array(rod_ids).reshape(COUNT,N),:3];grasp_segments=np.full(6,gi,dtype=int)
   outside=np.array([grasp[0],grasp[1],.026]);through=np.array([-.008,grasp[1]+.017,.026]);inside=through-np.array([0,0,.014]);clear=np.array([.13,inside[1],inside[2]])
   waypoints=[(3.8,grasp,.060),(4.5,grasp,.00199),(5.3,grasp+np.array([0,0,.014]),.00199),(6.3,outside,.00199),(9.0,through,.00199),(9.8,inside,.00199),(10.4,inside,.00199),(11.0,inside,.060),(12.2,clear,.060),(13.4,home,.060),(16,home,.060)]
   print('PICKUP_TARGET',grasp,'segments',grasp_segments,flush=True)
 theta=theta_for_gap(width);qj=ik(goal_flange(center,theta),qj);nextrobot=packed(robot_poses(qj,theta));robot_prev.assign(lastrobot);robot_next.assign(nextrobot);substep.zero_();wp.capture_launch(cap.graph);lastrobot=nextrobot
 q=s0.body_q.numpy().copy();assert np.isfinite(q).all();poses.append(q);counts.append(int(contacts.rigid_contact_count.numpy()[0]));jhistory.append(qj.copy());widths.append(width);targets.append(center.copy())
 if frame%60==0:
  np.savez_compressed(ROOT/'checkpoint.npz',poses=poses,fps=60,segments=N,segment_length=DS,clip_top_body=top,robot_start=robot_start,robot_names=robot_names,joint_positions=jhistory,jaw_width=widths,tcp_targets=targets,rod_ids=rod_ids,grasp_segments=grasp_segments)
  print('FRAME',frame,'clip',Rotation.from_quat(q[top,3:]).as_euler('xyz',degrees=True)[1],'wall',time.time()-start,flush=True)
np.savez_compressed(ROOT/(args.output+'.npz'),poses=poses,fps=60,segments=N,segment_length=DS,clip_top_body=top,robot_start=robot_start,robot_names=robot_names,joint_positions=jhistory,jaw_width=widths,tcp_targets=targets,rod_ids=rod_ids,contact_counts=counts,grasp=grasp,grasp_segments=grasp_segments,grasp_segment=gi if grasp is not None else -1)
report={'solver':'Newton SolverVBD / AVBD compliant ALM; native Newton collision detection','robot':'UR5 CB3, nominal URDF kinematics','gripper':'Robotiq 2F-85, four-bar closure solved kinematically','robot_control':'Hardcoded bundle-position tracking during approach, then scripted Cartesian insertion; numerical IK, kinematic links, no motor dynamics','grasp':'Contact and Coulomb friction only; no cable attachments to fingers or direct cable forces','connector':'Initially seated; fixed kinematic boundary, models a locked connector','cable_count':6,'length_mm':457.2,'diameter_mm':2,'bend_rigidity_Nm2':EI,'rest_curvature':'zero','distal_boundary':'free, hanging beyond table edge','physics_hz':3840,'iterations':60,'pad_friction':1.,'closed_jaw_gap_mm':1.99,'clip_spring_stiffness_Nm_rad':CLIP_SPRING_MULTIPLIER*.0002*180/math.pi,'clip_spring_damping_Nms_rad':CLIP_SPRING_MULTIPLIER*.000005*180/math.pi,'clip_preload_Nm':.03,'lengthwise_pull_mm':17,'insertion_tcp_height_mm':26,'seat_tcp_height_mm':12,'release_start_seconds':10.4,'release_complete_seconds':11.0,'retreat':'Withdraw along +X at fixed height until fingers clear the bundle, then lift to home','seconds':args.seconds,'runtime_seconds':time.time()-start}
(ROOT/(args.output+'.json')).write_text(json.dumps(report,indent=2)+'\n');print('DONE',flush=True)
