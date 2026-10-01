"""Contact-driven insertion into a native Newton hinged receiver.
Only a force/torque hand controller acts on the plug; the receiver is not animated.
"""
from pathlib import Path
import argparse,json,math,time
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import warp as wp
import newton
from assets import PIVOT,OPEN_DEG,SEAT,plug_parts,fixture_parts
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=7);p.add_argument('--output',default='motion');p.add_argument('--no-receiver-contact',action='store_true');args=p.parse_args()
wp.init();wp.set_device('cuda:0')
N=61;COUNT=6;L=.3048;DS=L/N;R=.001;PITCH=.0025;EI=.0011459155902616466
angle=math.radians(OPEN_DEG);rot0=Rotation.from_euler('x',angle);initial=PIVOT+rot0.apply(SEAT)+np.array([0,.008,.050])
far=np.array([0,.242,.010]);farrot=Rotation.from_euler('z',180,degrees=True)
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
b.add_shape_box(body=-1,hx=.19,hy=.24,hz=.008,xform=wp.transform(wp.vec3(0,.12,-.010),wp.quat_identity()),cfg=contact)
for center,extent in [((-.113, .12,-.0005),(.154,.48,.003)),((.113,.12,-.0005),(.154,.48,.003)),((0,-.078,-.0005),(.072,.084,.003)),((0,.198,-.0005),(.072,.324,.003))]:
 b.add_shape_box(body=-1,hx=extent[0]/2,hy=extent[1]/2,hz=extent[2]/2,xform=wp.transform(wp.vec3(*center),wp.quat_identity()),cfg=contact)
static,moving=fixture_parts();add_parts(static,-1)
receiver=b.add_link(xform=wp.transform(wp.vec3(*PIVOT),wp.quat(*rot0.as_quat())),mass=.025,com=wp.vec3(0,.018,0),inertia=wp.mat33(*np.diag(np.array([.040**2+.010**2,.050**2+.010**2,.050**2+.040**2])*.025/12).reshape(-1)),label='HingedReceiver')
receiver_shapes=add_parts(moving,receiver)
j=b.add_joint_revolute(parent=-1,child=receiver,parent_xform=wp.transform(wp.vec3(*PIVOT),wp.quat_identity()),child_xform=wp.transform_identity(),axis=wp.vec3(1,0,0),target_ke=0.,target_kd=0.,limit_lower=0.,limit_upper=math.radians(45),collision_filter_parent=False);b.add_articulation([j]);b.joint_q[b.joint_q_start[j]]=angle
plug=b.add_link(xform=wp.transform(wp.vec3(*initial),wp.quat(*rot0.as_quat())),mass=.015,inertia=wp.mat33(*np.diag(np.array([.027**2+.008**2,.042**2+.008**2,.042**2+.027**2])*.015/12).reshape(-1)),label='CableEndPlug')
plug_shapes=add_parts(plug_parts(),plug)
add_parts(plug_parts(),-1,far,farrot)
# Native rods follow an initially bent arc of exactly L in chord lengths.
a0=initial+rot0.apply([0,.009,.012]);a1=far+farrot.apply([0,.009,.012]);chord=a1-a0;distance=np.linalg.norm(chord);direction=chord/distance;up=np.array([0.,-direction[2],direction[1]])
total_angle=brentq(lambda t:DS*math.sin(t/2)/math.sin(t/(2*N))-distance,.001,5.8);radius=DS/(2*math.sin(total_angle/(2*N)));t=np.linspace(-total_angle/2,total_angle/2,N+1)
points=(a0+a1)/2+radius*np.sin(t)[:,None]*direction+(radius*(np.cos(t)-math.cos(total_angle/2)))[:,None]*up
cfg=newton.ModelBuilder.ShapeConfig(density=1000*DS/(DS+4*R/3),ke=1e6,kd=100.,mu=.5,gap=.0002)
rod_ids=[];attachment_points=[];initial_quats=[]
for c in range(COUNT):
 dx=(c-2.5)*PITCH;chain=[];joints=[];rots=[]
 for i in range(N):
  v=(points[i+1]-points[i])/DS;q=Rotation.from_euler('x',math.atan2(-v[1],v[2]));pos=(points[i]+points[i+1])/2+np.array([dx,0,0]);body=b.add_link(xform=wp.transform(wp.vec3(*pos),wp.quat(*q.as_quat())),label=f'Cable_{c}_{i:02}');chain.append(body);rots.append(q)
  b.add_shape_capsule(body=body,radius=R,half_height=DS/2,cfg=cfg)
  if i:joints.append(b.add_joint_rod(parent=chain[-2],child=body,parent_xform=wp.transform(wp.vec3(0,0,DS/2),wp.quat_identity()),child_xform=wp.transform(wp.vec3(0,0,-DS/2),wp.quat_identity()),stretch_stiffness=1e6,stretch_damping=.1,bend_stiffness=EI/DS,bend_damping=.005729577951308233*.005/DS))
 b.add_articulation(joints)
 parentq=(rot0.inv()*rots[0]).as_quat();j=b.add_joint_fixed(parent=plug,child=chain[0],parent_xform=wp.transform(wp.vec3(dx,.009,.012),wp.quat(*parentq)),child_xform=wp.transform(wp.vec3(0,0,-DS/2),wp.quat_identity()));b.add_articulation([j])
 anchor=a1+np.array([dx,0,0]);j=b.add_joint_fixed(parent=-1,child=chain[-1],parent_xform=wp.transform(wp.vec3(*anchor),wp.quat(*rots[-1].as_quat())),child_xform=wp.transform(wp.vec3(0,0,DS/2),wp.quat_identity()),collision_filter_parent=False);b.add_articulation([j]);rod_ids+=chain
if args.no_receiver_contact:
 for a in plug_shapes+[shape for body in rod_ids for shape in b.body_shapes[body]]:
  for c in receiver_shapes:b.shape_collision_filter_pairs.append((min(a,c),max(a,c)))
b.color(balance_colors=False);model=b.finalize(device='cuda:0');I=np.array(b.body_inertia).reshape(-1,3,3);ev=np.linalg.eigvalsh(I);assert ev.min()>0
model.body_inertia.assign(I.astype(np.float32));model.body_inv_inertia.assign(np.linalg.inv(I).astype(np.float32))
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=100000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=40,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=512,friction_epsilon=1e-4,rigid_joint_linear_ke=1e7,rigid_joint_angular_ke=1e5)
solver.joint_rod_rest_kb_local.zero_();solver.joint_rod_rest_twist.zero_()
assert not np.any(solver.joint_rod_rest_kb_local.numpy()) and not np.any(solver.joint_rod_rest_twist.numpy())
s0=model.state();s1=model.state();ctrl=model.control();contacts=pipeline.contacts();clock=wp.zeros(1,dtype=float);dt=1/3840
@wp.func
def smooth(v:float):
 u=wp.clamp(v,0.,1.);return u*u*(3.-2.*u)
@wp.kernel
def hand(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),force:wp.array(dtype=wp.spatial_vector),clock:wp.array(dtype=float)):
 t=clock[0];opening=0.4363323129985824
 qr=wp.transform_get_rotation(q[0]);ar=2.*wp.atan2(qr[0],qr[3]);w=wp.spatial_bottom(qd[0])
 # Time-independent bistable detent: stable around closed and open positions.
 torque=-0.012*wp.sin(6.28318530718*ar/opening)-0.001*w[0]
 force[0]=wp.spatial_vector(wp.vec3(0.),wp.vec3(torque,0.,0.))
 if t<5.2:
  a=opening*(1.-smooth((t-2.7)/1.5));target_rot=wp.quat_from_axis_angle(wp.vec3(1.,0.,0.),a)
  gap=.050*(1.-smooth((t-.5)/1.8))-.0003*smooth((t-2.3)/.4)
  target=wp.vec3(0.,-.018,.024)+wp.quat_rotate(target_rot,wp.vec3(0.,.018,.0065))+wp.vec3(0.,.008*(1.-smooth((t-.5)/1.0)),gap)
  pos=wp.transform_get_translation(q[1]);v=wp.spatial_top(qd[1]);f=4000.*(target-pos)-8.*v+wp.vec3(0.,0.,.14715)
  f=wp.vec3(wp.clamp(f[0],-20.,20.),wp.clamp(f[1],-20.,20.),wp.clamp(f[2],-20.,20.))
  e=target_rot*wp.quat_inverse(wp.transform_get_rotation(q[1]));sgn=1.
  if e[3]<0.:sgn=-1.
  tau=1.0*sgn*wp.vec3(e[0],e[1],e[2])-.003*wp.spatial_bottom(qd[1]);tau=wp.vec3(wp.clamp(tau[0],-.15,.15),wp.clamp(tau[1],-.15,.15),wp.clamp(tau[2],-.15,.15))
  fade=1.-smooth((t-4.8)/.4);force[1]=wp.spatial_vector(f*fade,tau*fade)
@wp.kernel
def tick(clock:wp.array(dtype=float),dt:float):clock[0]+=dt
with wp.ScopedCapture() as cap:
 for _ in range(64):
  s0.clear_forces();wp.launch(hand,1,inputs=[s0.body_q,s0.body_qd,s0.body_f,clock]);pipeline.collide(s0,contacts);solver.step(s0,s1,ctrl,contacts,dt);wp.launch(tick,1,inputs=[clock,dt]);s0,s1=s1,s0
poses=[];counts=[];start=time.time()
for frame in range(round(args.seconds*60)):
 wp.capture_launch(cap.graph);q=s0.body_q.numpy().copy();assert np.isfinite(q).all();poses.append(q);counts.append(int(contacts.rigid_contact_count.numpy()[0]))
 if frame%60==0:print('FRAME',frame,'receiver_angle',Rotation.from_quat(q[0,3:]).as_euler('xyz',degrees=True)[0],'plug_mm',q[1,:3]*1000,'wall',time.time()-start,flush=True)
np.savez_compressed(ROOT/(args.output+'.npz'),poses=poses,fps=60,segments=N,segment_length=DS,far_position=far,contact_counts=counts,initial_cable_endpoints=np.array([a0,a1]))
report={'engine':'Newton 1.6 / Warp 1.17','solver':'SolverVBD / AVBD, compliant ALM','physics_hz':3840,'iterations':40,'length_mm':304.8,'diameter_mm':2,'cable_count':6,'bend_rigidity_Nm2':EI,'rod_rest_curvature':'Explicit zero bend/twist solver caches; initially bent state is not material rest' ,'plug_mass_kg':.015,'receiver_mass_kg':.025,'hinge_axis':'X','hinge_limits_degrees':[0,45],'detent_torque_amplitude_Nm':.012,'detent_angular_damping_Nms_per_rad':.001,'detent_open_degrees':25,'detent_model':'tau=-.012*sin(2*pi*angle/open_angle)-.001*angular_velocity; no time-dependent receiver target','hand_position_stiffness_N_m':4000,'hand_position_damping_Ns_m':8,'hand_force_cap_N_per_axis':20,'hand_angular_stiffness_small_angle_Nm_rad':.5,'hand_angular_damping_Nms_rad':.003,'hand_torque_cap_Nm_per_axis':.15,'hand_release_seconds':5.2,'seconds':args.seconds,'no_receiver_contact':args.no_receiver_contact,'control_filter_scope':'Plug and all cable shapes against the receiver','runtime_seconds':time.time()-start}
(ROOT/(args.output+'.json')).write_text(json.dumps(report,indent=2)+'\n');print('DONE',flush=True)
