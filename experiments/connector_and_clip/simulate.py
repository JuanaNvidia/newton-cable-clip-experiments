"""Contact-driven insertion into a native Newton fixed receiver.
A force-driven plug hand and two cable grips combine seating and clip routing.
"""
from pathlib import Path
import argparse,json,math,time
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import warp as wp
import newton
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,HOLE_TOP,clip_meshes
from assets import RECEIVER_ORIGIN,OPEN_DEG,SEAT,plug_parts,fixture_parts
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=14);p.add_argument('--output',default='motion');p.add_argument('--no-receiver-contact',action='store_true');p.add_argument('--no-clip-contact',action='store_true');p.add_argument('--no-retention',action='store_true');p.add_argument('--original-clip-spring',action='store_true');args=p.parse_args()
wp.init();wp.set_device('cuda:0')
CLIP_SPRING_MULTIPLIER=1. if args.original_clip_spring else 10.
CLIP_PRELOAD=0. if args.original_clip_spring else .03
N=91;COUNT=6;L=.4572;DS=L/N;R=.001;PITCH=.0025;EI=.0011459155902616466
angle=math.radians(OPEN_DEG);rot0=Rotation.from_euler('x',angle);initial=RECEIVER_ORIGIN+np.array([0,.0045,.0105])+rot0.apply([0,.0135,-.004])+np.array([0,.008,.050])
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
b.add_shape_box(body=-1,hx=.19,hy=.34,hz=.008,xform=wp.transform(wp.vec3(0,.22,-.010),wp.quat_identity()),cfg=contact)
for center,extent in [((-.113, .22,-.0005),(.154,.68,.003)),((.113,.22,-.0005),(.154,.68,.003)),((0,-.078,-.0005),(.072,.084,.003)),((0,.298,-.0005),(.072,.524,.003))]:
 b.add_shape_box(body=-1,hx=extent[0]/2,hy=extent[1]/2,hz=extent[2]/2,xform=wp.transform(wp.vec3(*center),wp.quat_identity()),cfg=contact)
static,moving=fixture_parts();add_parts(static,-1)
receiver=b.add_link(xform=wp.transform(wp.vec3(*RECEIVER_ORIGIN),wp.quat_identity()),is_kinematic=True,mass=.025,com=wp.vec3(0,.018,0),inertia=wp.mat33(*np.diag(np.array([.040**2+.010**2,.050**2+.010**2,.050**2+.040**2])*.025/12).reshape(-1)),label='FixedReceiver')
receiver_shapes=add_parts(moving,receiver)
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
# Grips use two material locations roughly 40 mm either side of clip centre.
midpoints=(points[:-1]+points[1:])/2
clip_center_y=CLIP_ORIGIN[1]-.015
grip_indices=[int(np.argmin(abs(midpoints[:,1]-y))) for y in [clip_center_y-.0404,clip_center_y+.0404]]
grip_ids=[rod_ids[c*N+i+o] for c in range(COUNT) for i in grip_indices for o in range(-3,4)]
grip_start=wp.zeros(len(grip_ids),dtype=wp.vec3);grip_ids_wp=wp.array(grip_ids,dtype=wp.int32)
print('GRIP_INDICES',grip_indices,'arc separation mm',(grip_indices[1]-grip_indices[0])*DS*1000,flush=True)
if args.no_receiver_contact:
 for a in plug_shapes+[shape for body in rod_ids for shape in b.body_shapes[body]]:
  for c in receiver_shapes:b.shape_collision_filter_pairs.append((min(a,c),max(a,c)))
b.color(balance_colors=False);model=b.finalize(device='cuda:0');I=np.array(b.body_inertia).reshape(-1,3,3);ev=np.linalg.eigvalsh(I);assert ev.min()>0
model.body_inertia.assign(I.astype(np.float32));inv_I=np.linalg.inv(I).astype(np.float32);inv_I[0]=0;model.body_inv_inertia.assign(inv_I)
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=100000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=40,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=512,friction_epsilon=1e-4,rigid_joint_linear_ke=1e7,rigid_joint_angular_ke=1e5)
solver.joint_rod_rest_kb_local.zero_();solver.joint_rod_rest_twist.zero_()
assert not np.any(solver.joint_rod_rest_kb_local.numpy()) and not np.any(solver.joint_rod_rest_twist.numpy())
latch=wp.zeros(1,dtype=wp.int32);latch_enabled=not(args.no_retention or args.no_receiver_contact)
s0=model.state();s1=model.state();ctrl=model.control();contacts=pipeline.contacts();clock=wp.zeros(1,dtype=float);dt=1/3840
@wp.func
def smooth(v:float):
 u=wp.clamp(v,0.,1.);return u*u*(3.-2.*u)
@wp.kernel
def hand(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),force:wp.array(dtype=wp.spatial_vector),clock:wp.array(dtype=float)):
 t=clock[0];opening=0.4363323129985824
 if t<12.2:
  a=opening*(1.-smooth((t-2.7)/1.5));target_rot=wp.quat_from_axis_angle(wp.vec3(1.,0.,0.),a)
  gap=.050*(1.-smooth((t-.5)/1.8))-.0003*smooth((t-2.3)/.4)
  target=wp.vec3(0.,-.0135,.0345)+wp.quat_rotate(target_rot,wp.vec3(0.,.0135,-.004))+wp.vec3(0.,.008*(1.-smooth((t-.5)/1.0)),gap)
  pos=wp.transform_get_translation(q[1]);v=wp.spatial_top(qd[1]);f=4000.*(target-pos)-8.*v+wp.vec3(0.,0.,.14715)
  f=wp.vec3(wp.clamp(f[0],-20.,20.),wp.clamp(f[1],-20.,20.),wp.clamp(f[2],-20.,20.))
  e=target_rot*wp.quat_inverse(wp.transform_get_rotation(q[1]));sgn=1.
  if e[3]<0.:sgn=-1.
  tau=4.0*sgn*wp.vec3(e[0],e[1],e[2])-.003*wp.spatial_bottom(qd[1]);tau=wp.vec3(wp.clamp(tau[0],-.15,.15),wp.clamp(tau[1],-.15,.15),wp.clamp(tau[2],-.15,.15))
  fade=1.-smooth((t-11.8)/.4);force[1]=force[1]+wp.spatial_vector(f*fade,tau*fade)
@wp.kernel
def cable_grips(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),force:wp.array(dtype=wp.spatial_vector),clock:wp.array(dtype=float),ids:wp.array(dtype=wp.int32),starts:wp.array(dtype=wp.vec3)):
 j=wp.tid();t=clock[0];body=ids[j]
 if t<.0001:starts[j]=wp.transform_get_translation(q[body])
 if t<11.2:
  # Lower outside the mouth, pull laterally through the sprung gate, then release.
  side=float(((j//7)%2)*2-1);dx=(float(j//14)-2.5)*.0025;along=float(j%7-3)*.005024175824175824
  target=starts[j]+wp.vec3(.037*smooth(t/2.),0.,-.065*smooth((t-.5)/1.8))
  lower=smooth((t-5.5)/2.0)
  target=target*(1.-lower)+wp.vec3(.037+dx,0.200+side*.0447+along,.021)*lower
  pull=smooth((t-7.7)/2.0)
  target[0]-=.037*pull
  target[2]-=.006*smooth((t-9.4)/.8)
  pos=wp.transform_get_translation(q[body]);v=wp.spatial_top(qd[body])
  f=600.*(target-pos)-.12*v
  f=wp.vec3(wp.clamp(f[0],-.5,.5),wp.clamp(f[1],-.5,.5),wp.clamp(f[2],-.5,.5))
  force[body]=wp.spatial_vector(f*(1.-smooth((t-10.8)/.4)),wp.vec3(0.))
@wp.kernel
def plug_retention(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),force:wp.array(dtype=wp.spatial_vector),latch:wp.array(dtype=wp.int32)):
 # Approximate concealed lock: bounded, breakaway spring capture after seating.
 # No time trigger, body teleport, fixed attachment, or receiver motion.
 pos=wp.transform_get_translation(q[1]);delta=wp.vec3(0.,0.,.0305)-pos
 rot=wp.transform_get_rotation(q[1]);e=wp.quat_inverse(rot);sgn=1.
 if e[3]<0.:sgn=-1.
 error=sgn*wp.vec3(e[0],e[1],e[2])
 if latch[0]==0 and wp.abs(delta[0])<.001 and wp.abs(delta[1])<.001 and wp.abs(delta[2])<.0004 and wp.length(error)<.02618:latch[0]=1
 if latch[0]==1:
  if wp.length(delta)>.004 or wp.length(error)>.10453:latch[0]=2
  else:
   f=8000.*delta-6.*wp.spatial_top(qd[1]);f=f*wp.min(1.,5./wp.max(wp.length(f),1e-8))
   tau=12.*error-.008*wp.spatial_bottom(qd[1]);tau=tau*wp.min(1.,.2/wp.max(wp.length(tau),1e-8))
   force[1]=force[1]+wp.spatial_vector(f,tau)
@wp.kernel
def clip_preload(force:wp.array(dtype=wp.spatial_vector),body:int,torque:float):
 # Constant spring preload toward the closed stop. Newton clamps drive targets
 # to joint limits, so the preload is an explicit, time-independent torque.
 force[body]=force[body]+wp.spatial_vector(wp.vec3(0.),wp.vec3(0.,torque,0.))
@wp.kernel
def tick(clock:wp.array(dtype=float),dt:float):clock[0]+=dt
with wp.ScopedCapture() as cap:
 for _ in range(64):
  s0.clear_forces();wp.launch(clip_preload,1,inputs=[s0.body_f,top,CLIP_PRELOAD])
  if latch_enabled:wp.launch(plug_retention,1,inputs=[s0.body_q,s0.body_qd,s0.body_f,latch])
  wp.launch(hand,1,inputs=[s0.body_q,s0.body_qd,s0.body_f,clock]);wp.launch(cable_grips,len(grip_ids),inputs=[s0.body_q,s0.body_qd,s0.body_f,clock,grip_ids_wp,grip_start]);pipeline.collide(s0,contacts);solver.step(s0,s1,ctrl,contacts,dt);wp.launch(tick,1,inputs=[clock,dt]);s0,s1=s1,s0
poses=[];counts=[];latch_history=[];start=time.time()
for frame in range(round(args.seconds*60)):
 wp.capture_launch(cap.graph);q=s0.body_q.numpy().copy();assert np.isfinite(q).all();poses.append(q);latch_history.append(int(latch.numpy()[0]));counts.append(int(contacts.rigid_contact_count.numpy()[0]))
 if frame%120==0:np.savez_compressed(ROOT/'checkpoint.npz',latch_state=latch_history,poses=poses,fps=60,segments=N,segment_length=DS,far_position=far,initial_cable_endpoints=np.array([a0,a1]),clip_origin=CLIP_ORIGIN,clip_top_body=top,grip_indices=grip_indices,grip_ids=grip_ids)
 if frame%60==0:print('CLIP',Rotation.from_quat(q[top,3:]).as_euler('xyz',degrees=True)[1], 'FRAME',frame,'receiver_angle',Rotation.from_quat(q[0,3:]).as_euler('xyz',degrees=True)[0],'plug_mm',q[1,:3]*1000,'wall',time.time()-start,flush=True)
np.savez_compressed(ROOT/(args.output+'.npz'),latch_state=latch_history,clip_origin=CLIP_ORIGIN,clip_top_body=top,grip_indices=grip_indices,grip_ids=grip_ids,poses=poses,fps=60,segments=N,segment_length=DS,far_position=far,contact_counts=counts,initial_cable_endpoints=np.array([a0,a1]))
report={'engine':'Newton 1.6 / Warp 1.17','solver':'SolverVBD / AVBD, compliant ALM','physics_hz':3840,'iterations':40,'length_mm':457.2,'diameter_mm':2,'cable_count':6,'bend_rigidity_Nm2':EI,'rod_rest_curvature':'Explicit zero bend/twist solver caches; initially bent state is not material rest' ,'plug_mass_kg':.015,'receiver_motion':'Fixed kinematic body, no joint or applied force','plug_path':'Leading edge engages, then hand rotates plug from 25 degrees to flat','lock_status':'Approximate pose-triggered, bounded breakaway spring capture; concealed latch geometry not reconstructed','retention_enabled':latch_enabled,'retention_position_stiffness_N_m':8000,'retention_damping_Ns_m':6,'retention_force_cap_N':5,'retention_angular_stiffness_Nm_rad':6,'retention_angular_damping_Nms_rad':.008,'retention_torque_cap_Nm':.2,'retention_engagement_tolerance_mm':[1,1,.4],'retention_engagement_angle_deg':3,'retention_breakaway_displacement_mm':4,'retention_breakaway_angle_deg':12,'hand_position_stiffness_N_m':4000,'hand_position_damping_Ns_m':8,'hand_force_cap_N_per_axis':20,'hand_angular_stiffness_small_angle_Nm_rad':2.,'hand_angular_damping_Nms_rad':.003,'hand_torque_cap_Nm_per_axis':.15,'hand_release_seconds':12.2,'cable_grip_release_seconds':11.2,'grip_stiffness_N_m':600,'grip_damping_Ns_m':.12,'grip_force_cap_N_per_axis_per_sample':.5,'grip_samples_per_cable_per_hand':7,'grip_patch_length_mm':6*DS*1000,'grip_target_y_separation_mm':89.4,'grip_lateral_pull_mm':37,'grip_support_from_seconds':0,'grip_initial_support_lowering_mm':65,'clip_scale':1.,'clip_hinge_stiffness_Nm_rad':CLIP_SPRING_MULTIPLIER*.0002*180/math.pi,'clip_hinge_damping_Nms_rad':CLIP_SPRING_MULTIPLIER*.000005*180/math.pi,'clip_closing_preload_Nm':CLIP_PRELOAD,'original_clip_spring':args.original_clip_spring,'clip_hinge_limits_deg':[-75,0],'no_clip_contact':args.no_clip_contact,'seconds':args.seconds,'no_receiver_contact':args.no_receiver_contact,'control_filter_scope':'Plug and all cable shapes against the receiver','runtime_seconds':time.time()-start}
(ROOT/(args.output+'.json')).write_text(json.dumps(report,indent=2)+'\n');print('DONE',flush=True)
