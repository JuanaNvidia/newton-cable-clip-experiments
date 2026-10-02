"""UR5/Robotiq contact-only pickup and insertion of six Newton cables."""
from pathlib import Path
from retention import retained_at_clip,crossings
from scene import TABLE_BOXES
import argparse,json,math,time
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import warp as wp
import newton
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,HOLE_TOP,clip_meshes
from robot import ARM_NAMES,GRIP_NAMES,poses as robot_poses,packed,ik,goal_flange,theta_for_gap,gripper_local,TOOL_ROT
from assets import RECEIVER_ORIGIN,OPEN_DEG,SEAT,plug_parts,fixture_parts
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=37);p.add_argument('--output',default='motion');p.add_argument('--no-clip-contact',action='store_true');args=p.parse_args()
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
# Continuous tabletop beneath and well beyond the clip.
for center,size in TABLE_BOXES:
 b.add_shape_box(body=-1,hx=size[0]/2,hy=size[1]/2,hz=size[2]/2,xform=wp.transform(wp.vec3(*center),wp.quat_identity()),cfg=contact)
static,moving=fixture_parts();add_parts(static,-1)
receiver=b.add_link(xform=wp.transform(wp.vec3(*RECEIVER_ORIGIN),wp.quat_identity()),is_kinematic=True,mass=.025,com=wp.vec3(0,.018,0),inertia=wp.mat33(*np.diag(np.array([.040**2+.010**2,.050**2+.010**2,.050**2+.040**2])*.025/12).reshape(-1)),label='FixedReceiver')
receiver_shapes=add_parts(moving,receiver)
plug=b.add_link(is_kinematic=True,xform=wp.transform(wp.vec3(*initial),wp.quat(*rot0.as_quat())),mass=.015,inertia=wp.mat33(*np.diag(np.array([.027**2+.008**2,.042**2+.008**2,.042**2+.027**2])*.015/12).reshape(-1)),label='CableEndPlug')
plug_shapes=add_parts(plug_parts(),plug)

# Straight material rods in a bent initial configuration, connector already locked.
# The distal connector is a free dynamic rigid body, not a table anchor.
a0=initial+np.array([0,.009,.012])
channel_z=CLIP_ORIGIN[2]+.013
feed_slope=(channel_z-a0[2])/(CLIP_ORIGIN[1]-.015-a0[1])
def insertion_target(x,radius2):
 dy=math.sqrt((radius2-(x-a0[0])**2)/(1+feed_slope**2))
 return np.array([x,a0[1]+dy,a0[2]+feed_slope*dy])
u=np.linspace(0,1,10001);yy=.44*u
xx=.30*yy
zz=.0425+.03*np.sin(np.pi*yy/.44)-.020*yy/.44
curve=np.column_stack([xx,.009+yy,zz]);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(curve,axis=0),axis=1))]
assert arc[-1]>=L, 'Initial curve must cover the full cable material length'
points=np.column_stack([np.interp(np.linspace(0,L,N+1),arc,curve[:,k]) for k in range(3)])
initial_lengths=np.linalg.norm(np.diff(points,axis=0),axis=1);assert np.max(np.abs(initial_lengths-DS))<1e-7
(ROOT/'initialization_check.json').write_text(json.dumps({'target_length_mm':L*1000,'sample_curve_length_mm':float(arc[-1]*1000),'min_initial_segment_mm':float(initial_lengths.min()*1000),'max_initial_segment_mm':float(initial_lengths.max()*1000),'target_segment_mm':DS*1000,'passed':True},indent=2)+'\n')
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
# One shared dynamic plug ties all six distal ends together.
farrot=Rotation.from_euler('z',180,degrees=True)
far=a1-farrot.apply([0,.009,.012])
distal=b.add_link(xform=wp.transform(wp.vec3(*far),wp.quat(*farrot.as_quat())),mass=.015,inertia=wp.mat33(*np.diag(np.array([.027**2+.008**2,.042**2+.008**2,.042**2+.027**2])*.015/12).reshape(-1)),label='FreeDistalConnector')
add_parts(plug_parts(),distal)
distal_anchors=[]
for c in range(COUNT):
 body=rod_ids[c*N+N-1];dx=(c-2.5)*PITCH
 local=farrot.inv().apply(a1+np.array([dx,0,0])-far);distal_anchors.append(local)
 q=Rotation.from_quat(np.asarray(b.body_q[body])[3:])
 j=b.add_joint_fixed(parent=distal,child=body,parent_xform=wp.transform(wp.vec3(*local),wp.quat(*(farrot.inv()*q).as_quat())),child_xform=wp.transform(wp.vec3(0,0,DS/2),wp.quat_identity()))
 b.add_articulation([j])
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
PICKUP_YAW=-math.atan(.30)
open_theta=theta_for_gap(.030);hold_theta=theta_for_gap(.00599)
seed=np.array([-2.42750698,-1.55334897,2.38709924,-.83375027,-3.9983033,1.57079633])
home=np.array([.13,.26,.20]);qj=ik(goal_flange(home,open_theta,Rotation.from_euler('z',PICKUP_YAW)*TOOL_ROT),seed)
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
  lip=fingertip_lip();robot_shapes.append(b.add_shape_mesh(body=body,mesh=newton.Mesh(lip.vertices,lip.faces.flatten()).compute_convex_hull(),cfg=padcfg,label=name+'_support_lip'))
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
solver=newton.solvers.SolverVBD(model,iterations=80,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=512,friction_epsilon=1e-4,rigid_joint_linear_ke=1e7,rigid_joint_angular_ke=1e5)
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
insert_x=-.008;entry_reached=False;entry_time=None
recovery_triggered=False;recovery_grasp=None;recovery_indices=np.full(6,-1,dtype=int);recovery_yaw=0.;recovery_x=None;recovery_pull_mm=0.;recovery_hold_count=0;recovery_goal_reached=False;recovery_goal_time=None;recovery_detection=None;recovery_retention=[];recovery_grasp_history=[]
poses=[];counts=[];jhistory=[];widths=[];targets=[];start=time.time();grasp=None;lastrobot=initial_robot;waypoints=None;grasp_segments=np.full(6,49,dtype=int)
def cable_center(q):
 # Follow the same material location, avoiding ambiguous intersections when
 # the bundle bends back through the pickup plane.
 chains=q[np.array(rod_ids).reshape(COUNT,N),:3]
 point=chains[:,gi,:].mean(axis=0);point[1]=max(.240,point[1]);return point

for frame in range(round(args.seconds*60)):
 t=frame/60
 if frame==90:
  stateq=s0.body_q.numpy();chains=stateq[np.array(rod_ids).reshape(COUNT,N),:3]
  gi=int(np.argmin(abs(chains[:,:,1].mean(axis=0)-.244)));grasp=chains[:,gi,:].mean(axis=0)
  # The downstream grasp captures the bundle above the physical support lips.
  print('GRASP',grasp,'spread',np.ptp(chains[:,gi,:],axis=0),'segment',gi,flush=True)
  above=grasp+np.array([0,0,.075]);outside=np.array([grasp[0],grasp[1],a0[2]+feed_slope*(grasp[1]-a0[1])]);tension=outside+np.array([0,.017,feed_slope*.017]);radius2=np.sum((tension-a0)**2);through=insertion_target(-.008,radius2);inside=through.copy()
  waypoints=[(1.5,home,.030),(2.8,above,.030),(3.8,grasp,.030),(4.5,grasp,.00599),(5.3,grasp+np.array([0,0,.014]),.00599),(6.3,outside,.00599),(7.5,tension,.00599),(10.0,through,.00599),(12.5,through,.00599),(13.5,inside,.00599),(14.0,inside,.00599),(14.8,inside,.085),(18,inside,.085)]
 center=home.copy();width=.030
 if waypoints:
  for a,z in zip(waypoints[:-1],waypoints[1:]):
   u=float(np.clip((t-a[0])/(z[0]-a[0]),0,1));u=u*u*(3-2*u);center=(1-u)*a[1]+u*z[1];width=(1-u)*a[2]+u*z[2]
   if t<=z[0]:break
 # Scripted state feedback during approach follows the still-swinging
 # bundle at a fixed Y plane. It commands only robot poses, never cable forces.
 if 1.5<=t<=3.8:
  current=s0.body_q.numpy();tracked=cable_center(current)
  if t<2.8:
   u=np.clip((t-1.5)/1.3,0,1);u=u*u*(3-2*u);center=home*(1-u)+(tracked+np.array([0,0,.075]))*u
  elif t<3.8:
   u=np.clip((t-2.8),0,1);u=u*u*(3-2*u);center=tracked+np.array([0,0,.075*(1-u)])
  else:center=tracked
  if frame==228:
   grasp=center.copy();chains=current[np.array(rod_ids).reshape(COUNT,N),:3];grasp_segments=np.full(6,gi,dtype=int)
   outside=np.array([grasp[0],grasp[1],a0[2]+feed_slope*(grasp[1]-a0[1])]);tension=outside+np.array([0,.017,feed_slope*.017]);radius2=np.sum((tension-a0)**2);through=insertion_target(-.008,radius2);inside=through.copy()
   waypoints=[(3.8,grasp,.030),(4.5,grasp,.00599),(5.3,grasp+np.array([0,0,.014]),.00599),(6.3,outside,.00599),(7.5,tension,.00599),(10.0,through,.00599),(12.5,through,.00599),(13.5,inside,.00599),(14.0,inside,.00599),(14.8,inside,.085),(18,inside,.085)]
   print('PICKUP_TARGET',grasp,'segments',grasp_segments,flush=True)
 if 10.0<=t<12.5:
  current=s0.body_q.numpy();all_inside=True
  for cable in range(COUNT):
   rq=current[np.array(rod_ids).reshape(COUNT,N)[cable]];directions=Rotation.from_quat(rq[:,3:]).apply(np.tile([0,0,1],(N,1)));starts=rq[:,:3]-directions*DS/2;ends=rq[:,:3]+directions*DS/2
   ix=np.where((starts[:,1]-.200)*(ends[:,1]-.200)<=0)[0];crossings=[starts[k]+(ends[k]-starts[k])*(.200-starts[k,1])/(ends[k,1]-starts[k,1]) for k in ix if abs(ends[k,1]-starts[k,1])>1e-9]
   point=min(crossings,key=lambda v:abs(v[0])) if crossings else None
   if point is None:all_inside=False;continue
   lidlocal=Rotation.from_quat(current[top,3:]).inv().apply(point-current[top,:3]);local=point-CLIP_ORIGIN
   all_inside=all_inside and .004<local[0]<.027 and local[2]>.0118 and -.002<lidlocal[0]<.023 and lidlocal[2]<.0163
  if all_inside:
   if not entry_reached:print('ENTRY_REACHED',t,'TCP_X',insert_x,flush=True);entry_time=t
   entry_reached=True
  else:insert_x=max(-.040,insert_x-.012/60)
 if 10.0<=t:center[0]=insert_x
 if 7.5<=t:
  center=insertion_target(center[0],radius2)
 if t>=15.5:
  seated=insertion_target(insert_x,radius2);raised=seated+np.array([0,0,.10])
  retreat=[(15.5,seated),(17.0,raised),(18.0,raised),(19.5,home),(22.0,home)]
  for aa,bb in zip(retreat[:-1],retreat[1:]):
   u=float(np.clip((t-aa[0])/(bb[0]-aa[0]),0,1));u=u*u*(3-2*u);center=(1-u)*aa[1]+u*bb[1]
   if t<=bb[0]:break
  width=.085
 roll=0.
 yaw=PICKUP_YAW if t<7.5 else PICKUP_YAW*(1-float(np.clip((t-7.5)/2.5,0,1)))
 # After the first release and withdrawal, regrasp on the seated-connector side
 # only if the same geometric retention criterion used by validation fails.
 if frame==1080:
  current=s0.body_q.numpy();mask,_,_=retained_at_clip(current,rod_ids,DS,top);recovery_detection=mask.tolist();recovery_triggered=not bool(mask.all())
  print('RECOVERY_CHECK',t,mask.tolist(),flush=True)
  if recovery_triggered:
   pts,recovery_indices,tangents=crossings(current,rod_ids,DS,.135)
   assert (recovery_indices>=0).all() and np.isfinite(pts).all()
   recovery_yaw=-math.atan2(float(tangents[:,0].mean()),float(tangents[:,1].mean()))
   recovery_grasp=np.array([(pts[:,0].min()+pts[:,0].max())/2,.135,pts[:,2].min()-.0015])
   recovery_start=center.copy()
 if recovery_triggered and t>=18.0:
  current=s0.body_q.numpy()
  if t<=20.5:
   ids=np.array(rod_ids).reshape(COUNT,N);pts=current[ids[np.arange(6),recovery_indices],:3]
   tracked=np.array([(pts[:,0].min()+pts[:,0].max())/2,pts[:,1].mean(),pts[:,2].min()-.0015])
   above=tracked+np.array([0,0,.075])
   if t<19.2:
    u=float(np.clip((t-18.0)/1.2,0,1));u=u*u*(3-2*u);center=(1-u)*recovery_start+u*above;yaw=u*recovery_yaw
   else:
    u=float(np.clip((t-19.2)/1.3,0,1));u=u*u*(3-2*u);center=tracked+np.array([0,0,.075*(1-u)]);yaw=recovery_yaw
   width=(1-u)*.085+u*.040 if t<19.2 else .040
   if frame==1230:
    recovery_grasp=center.copy()
    recovery_feed=recovery_grasp.copy();recovery_feed[2]=max(recovery_grasp[2]+.003,a0[2]+feed_slope*(recovery_grasp[1]-a0[1])-.003)
    recovery_radius2=np.sum((recovery_feed-a0)**2);recovery_x=float(recovery_feed[0]);recovery_initial_x=recovery_x
    print('RECOVERY_GRASP',recovery_grasp,'segments',recovery_indices,'yaw',math.degrees(recovery_yaw),flush=True)
  elif t<22.3:
   points=[(20.5,recovery_grasp,.040),(21.3,recovery_grasp,.00599),(22.3,recovery_feed,.00599)]
   for aa,bb in zip(points[:-1],points[1:]):
    u=float(np.clip((t-aa[0])/(bb[0]-aa[0]),0,1));u=u*u*(3-2*u);center=(1-u)*aa[1]+u*bb[1];width=(1-u)*aa[2]+u*bb[2]
    if t<=bb[0]:break
   yaw=recovery_yaw
  else:
   mask,_,_=retained_at_clip(current,rod_ids,DS,top)
   if 22.3<=t<28.3:
    if mask.all():
     recovery_hold_count+=1
     if recovery_hold_count>=15 and not recovery_goal_reached:
      recovery_goal_reached=True;recovery_goal_time=t;print('RECOVERY_ALL_INSIDE',t,'pull_mm',recovery_pull_mm,flush=True)
    else:
     recovery_hold_count=0
     # Stop advancing while all six are inside, resume if retention is lost.
     recovery_x=max(recovery_initial_x-.024,recovery_x-.004/60)
    recovery_pull_mm=(recovery_initial_x-recovery_x)*1000
   dy=math.sqrt(recovery_radius2-(recovery_x-a0[0])**2-(recovery_feed[2]-a0[2])**2)
   recovery_seat=np.array([recovery_x,a0[1]+dy,recovery_feed[2]])
   raised=recovery_seat+np.array([0,0,.10])
   points=[(22.3,recovery_seat,.00599),(28.3,recovery_seat,.00599),(29.1,recovery_seat,.085),(29.5,recovery_seat,.085),(31.0,raised,.085),(32.5,home,.085),(37.,home,.085)]
   for aa,bb in zip(points[:-1],points[1:]):
    u=float(np.clip((t-aa[0])/(bb[0]-aa[0]),0,1));u=u*u*(3-2*u);center=(1-u)*aa[1]+u*bb[1];width=(1-u)*aa[2]+u*bb[2]
    if t<=bb[0]:break
   yaw=recovery_yaw*(1-float(np.clip((t-31.0)/1.5,0,1)))
 mask,_,_=retained_at_clip(s0.body_q.numpy(),rod_ids,DS,top)
 recovery_retention.append(mask.tolist());recovery_grasp_history.append(recovery_indices.copy())
 theta=theta_for_gap(width);qj=ik(goal_flange(center,theta,Rotation.from_euler('z',yaw)*TOOL_ROT),qj);nextrobot=packed(robot_poses(qj,theta));robot_prev.assign(lastrobot);robot_next.assign(nextrobot);substep.zero_();wp.capture_launch(cap.graph);lastrobot=nextrobot
 q=s0.body_q.numpy().copy();assert np.isfinite(q).all();poses.append(q);counts.append(int(contacts.rigid_contact_count.numpy()[0]));jhistory.append(qj.copy());widths.append(width);targets.append(center.copy())
 if frame%60==0:
  np.savez_compressed(ROOT/(args.output+'_checkpoint.npz'),poses=poses,fps=60,segments=N,segment_length=DS,clip_top_body=top,robot_start=robot_start,robot_names=robot_names,distal_body=distal,distal_anchors=distal_anchors,joint_positions=jhistory,jaw_width=widths,tcp_targets=targets,rod_ids=rod_ids,grasp_segments=grasp_segments,recovery_triggered=recovery_triggered,recovery_indices=recovery_indices,recovery_retention=recovery_retention)
  print('FRAME',frame,'clip',Rotation.from_quat(q[top,3:]).as_euler('xyz',degrees=True)[1],'wall',time.time()-start,flush=True)
 # When recovery is unnecessary, finish after four seconds of observation.
 if frame>=1320 and not recovery_triggered:break
np.savez_compressed(ROOT/(args.output+'.npz'),poses=poses,fps=60,segments=N,segment_length=DS,clip_top_body=top,robot_start=robot_start,robot_names=robot_names,distal_body=distal,distal_anchors=distal_anchors,joint_positions=jhistory,jaw_width=widths,tcp_targets=targets,rod_ids=rod_ids,contact_counts=counts,grasp=grasp,grasp_segments=grasp_segments,grasp_segment=gi if grasp is not None else -1,recovery_triggered=recovery_triggered,recovery_indices=recovery_indices,recovery_retention=recovery_retention)
report={'fingertip_modification':'Physical wedge support lips: 2.5 mm inward projection, 0.10 mm leading-edge thickness, 1 mm back thickness, 22 mm width. Original pads remain.','scene_variant':'interior tabletop with second dynamic connector','distal_connector_mass_kg':.015,'table_bounds_xy_m':[[-.31,.30],[-.12,.52]],'tcp_beyond_pad_center_mm':17.70,'solver':'Newton SolverVBD / AVBD compliant ALM; native Newton collision detection','robot':'UR5 CB3, nominal URDF kinematics','gripper':'Robotiq 2F-85, four-bar closure solved kinematically','robot_control':'Hardcoded bundle-position tracking during approach, then scripted Cartesian insertion; numerical IK, kinematic links, no motor dynamics','grasp':'Contact and Coulomb friction only; no cable attachments to fingers or direct cable forces','connector':'Initially seated; fixed kinematic boundary, models a locked connector','cable_count':6,'length_mm':457.2,'diameter_mm':2,'bend_rigidity_Nm2':EI,'rest_curvature':'zero','distal_boundary':'All six ends fixed to one free dynamic 15 g connector; tabletop contact active','physics_hz':3840,'iterations':80,'pad_friction':1.,'approach_jaw_gap_mm':30,'pickup_roll_degrees':0,'transport_roll_degrees':0,'pickup_yaw_degrees':math.degrees(PICKUP_YAW),'closed_jaw_gap_mm':5.99,'clip_spring_stiffness_Nm_rad':CLIP_SPRING_MULTIPLIER*.0002*180/math.pi,'clip_spring_damping_Nms_rad':CLIP_SPRING_MULTIPLIER*.000005*180/math.pi,'clip_preload_Nm':.03,'initial_lateral_slope':0.30,'lengthwise_pull_mm':17,'target_clip_plane_height_mm':float(channel_z*1000),'insertion_tcp_height_mm':float(through[2]*1000),'seat_tcp_height_mm':float(insertion_target(insert_x,radius2)[2]*1000),'release_start_seconds':14.0,'release_complete_seconds':14.8,'retreat':'Open jaws then lift vertically 100 mm above the tabletop and return home','retreat_clear_seconds':18.0,'home_seconds':19.5,'straighten_before_insertion':True,'entry_feedback':'At clip center plane, all six centers within bottom-local X 4..27 mm and lid-local X -2..23 mm below its flat underside. This requires crossing the rotating entrance lip; extra lateral travel is capped at world X -40 mm','entry_goal_reached':bool(entry_reached),'entry_goal_time':entry_time,'final_grip_x_m':float(insert_x),'insertion_path':'Maintain connector-to-grip distance and target the straight cable chord at world Z 14 mm on the clip plane; all elastic deformation and contacts remain solved by Newton','through_tcp_y_m':float(through[1]),'seat_tcp_y_m':float(insertion_target(insert_x,radius2)[1]),'seconds':len(poses)/60,'maximum_requested_seconds':args.seconds,'runtime_seconds':time.time()-start}
report.update(recovery_enabled=True,recovery_triggered=bool(recovery_triggered),retention_before_recovery=recovery_detection,recovery_grasp_side='Upstream, seated-connector side of clip, nominal world Y 135 mm',recovery_pull_direction='Negative world X, constant distance from seated connector',recovery_pull_speed_mm_s=4.,recovery_max_pull_mm=24.,recovery_actual_pull_mm=float(recovery_pull_mm),recovery_goal_reached=bool(recovery_goal_reached),recovery_goal_time=recovery_goal_time,recovery_pickup_yaw_degrees=float(math.degrees(recovery_yaw)),recovery_grasp_position_m=None if recovery_grasp is None else recovery_grasp.tolist(),recovery_indices=recovery_indices.tolist(),first_release_start_seconds=14.,first_release_complete_seconds=14.8,first_retreat_clear_seconds=18.,recovery_close_start_seconds=20.5,recovery_close_end_seconds=21.3,recovery_lift_end_seconds=22.3,recovery_pull_end_seconds=28.3)
if recovery_triggered:
 report.update(release_start_seconds=28.3,release_complete_seconds=29.1,retreat_clear_seconds=31.,home_seconds=32.5,retreat='After opposite-side pull, open jaws and lift 100 mm, then return home',robot_control='First scripted insertion, retention check after release, then conditional opposite-side regrasp and bounded geometric-feedback pull')
(ROOT/(args.output+'.json')).write_text(json.dumps(report,indent=2)+'\n');print('DONE',flush=True)
