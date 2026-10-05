"""Sequential contact grasps; native Newton VBD cables and three spring clips."""
from pathlib import Path
import argparse,json,math,time
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import warp as wp
import newton
from clip_geometry import HOLE_BOTTOM,HOLE_TOP,clip_meshes
from scene import *
from robot import ARM_NAMES,GRIP_NAMES,poses as robot_poses,packed,ik,goal_flange,theta_for_gap,TOOL_ROT
from assets import SEAT,plug_parts,fixture_parts
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=80);p.add_argument('--output',default='motion');p.add_argument('--iterations',type=int,default=80);p.add_argument('--resume');p.add_argument('--retry-first',action='store_true');p.add_argument('--retry-start',type=float,default=16.5);p.add_argument('--recover-home',action='store_true');p.add_argument('--recovery-start',type=float);p.add_argument('--repair-clips',type=int,nargs='+',help='One-based clip indices for contact-only upstream recovery');args=p.parse_args()
wp.init();wp.set_device('cuda:0')
R=RADIUS;EI=.0011459155902616466
b=newton.ModelBuilder(gravity=(0.,0.,-9.81))
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
static,moving=fixture_parts()
for seat,yaw in zip(SOCKET_POSITIONS,SOCKET_YAWS):
 rot=Rotation.from_euler('z',yaw);receiver=seat-rot.apply(SEAT)
 add_parts(moving,-1,receiver,rot)
 # The reference base was authored around a receiver at (0,-18,24) mm.
 add_parts(static,-1,receiver-rot.apply([0,-.018,.024]),rot)
# Almost flat initial harness with outlet-height transitions near each end.
u=np.linspace(0,1,12001)
def curve(span):return np.column_stack([.13+.014*np.sin(2*np.pi*u),-.001+span*u,.0023+.0147*(np.exp(-u/.035)+np.exp(-(1-u)/.035))])
def length(span):return np.linalg.norm(np.diff(curve(span),axis=0),axis=1).sum()
span=brentq(lambda span:length(span)-L,.70,.77);guide=curve(span);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(guide,axis=0),axis=1))]
points=np.column_stack([np.interp(np.linspace(0,L,N+1),arc,guide[:,k]) for k in range(3)])
plugs=[];latches=[];plug_anchors=[];plug_rots=[Rotation.identity(),Rotation.from_euler('z',np.pi)]
for k,rot in enumerate(plug_rots):
 pos=points[0 if k==0 else -1]-rot.apply([0,.009,.012])
 body=b.add_link(xform=transform(pos,rot),mass=.015,inertia=wp.mat33(*np.diag([1.2e-6,2.2e-6,3.1e-6]).reshape(-1)),label=f'EndConnector_{k+1}');plugs.append(body);add_parts(plug_parts(),body)
 seatrot=Rotation.from_euler('z',SOCKET_YAWS[k])
 j=b.add_joint_fixed(parent=-1,child=body,parent_xform=transform(SOCKET_POSITIONS[k],seatrot),child_xform=transform([0,0,0]),collision_filter_parent=False,label=f'SeatLatch_{k+1}');b.add_articulation([j]);latches.append(j)
cfg=newton.ModelBuilder.ShapeConfig(density=1000*DS/(DS+4*R/3),ke=1e6,kd=100.,mu=.5,gap=.0002)
rod_ids=[];anchor_q=[]
for c in range(COUNT):
 dx=(c-2.5)*PITCH;chain=[];joints=[];rots=[]
 for i in range(N):
  v=points[i+1]-points[i];q=Rotation.align_vectors([v],[[0,0,1]])[0];pos=(points[i]+points[i+1])/2+[dx,0,0]
  body=b.add_link(xform=transform(pos,q),label=f'Cable_{c}_{i:03}');chain.append(body);rots.append(q);b.add_shape_capsule(body=body,radius=R,half_height=DS/2,cfg=cfg)
  if i:joints.append(b.add_joint_rod(parent=chain[-2],child=body,parent_xform=transform([0,0,DS/2]),child_xform=transform([0,0,-DS/2]),stretch_stiffness=1e6,stretch_damping=.1,bend_stiffness=EI/DS,bend_damping=.005729577951308233*.005/DS))
 b.add_articulation(joints)
 for k,rot in enumerate(plug_rots):
  anchor=np.array([dx if k==0 else -dx,.009,.012]);localq=rot.inv()*rots[0 if k==0 else -1]
  j=b.add_joint_fixed(parent=plugs[k],child=chain[0 if k==0 else -1],parent_xform=transform(anchor,localq),child_xform=transform([0,0,-DS/2 if k==0 else DS/2]));b.add_articulation([j]);plug_anchors.append(anchor);anchor_q.append(localq.as_quat())
 rod_ids+=chain
meshset=clip_meshes();tops=[]
def clip_shape(data,body,cfg,offset=None,rot=None,convex=False):
 v,f=data;m=newton.Mesh(v,f)
 if convex:m=m.compute_convex_hull()
 return b.add_shape_mesh(body=body,mesh=m,cfg=cfg,xform=transform([0,0,0] if offset is None else offset,rot))
for k,(origin,yaw) in enumerate(zip(CLIP_ORIGINS,CLIP_YAWS)):
 rot=Rotation.from_euler('z',yaw)
 clip_shape(meshset['bottom'],-1,contact,origin,rot);clip_shape(meshset['support'],-1,contact,origin,rot,True)
 top=b.add_link(xform=transform(origin+rot.apply(HOLE_BOTTOM-HOLE_TOP),rot),label=f'SpringClip_{k+1}');tops.append(top)
 cc=newton.ModelBuilder.ShapeConfig(density=1000.,ke=1e6,kd=100.,mu=.5,gap=.0002,margin=0.)
 for m in meshset['convex_top']:clip_shape(m,top,cc,convex=True)
 hinge=b.add_joint_revolute(parent=-1,child=top,parent_xform=transform(origin+rot.apply(HOLE_BOTTOM),rot),child_xform=transform(HOLE_TOP),axis=wp.vec3(0,1,0),target_pos=0.,target_ke=.114591559,target_kd=.002864789,limit_lower=math.radians(-75),limit_upper=0.,collision_filter_parent=False);b.add_articulation([hinge])
# Robot bodies are driven by URDF FK. Finger poses obey the four-bar loop.
# All cable forces are gravity, rod elasticity and Newton collision contacts.
import trimesh
robot_start=len(b.body_q);robot_names=ARM_NAMES+GRIP_NAMES
robot_ids=[];robot_shapes=[]
PICKUP_YAW=0.
open_theta=theta_for_gap(.030);hold_theta=theta_for_gap(.00599)
seed=np.array([-2.42750698,-1.55334897,2.38709924,-.83375027,-3.9983033,1.57079633])
home=np.array([.15,.38,.26]);qj=ik(goal_flange(home,open_theta,Rotation.from_euler('z',PICKUP_YAW)*TOOL_ROT),seed)
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
model.body_inertia.assign(I.astype(np.float32));inv_I=np.linalg.inv(I).astype(np.float32);inv_I[robot_ids]=0;model.body_inv_inertia.assign(inv_I)
enabled=model.joint_enabled.numpy();enabled[latches]=False;model.joint_enabled.assign(enabled)
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=150000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=args.iterations,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=512,friction_epsilon=1e-4,rigid_joint_linear_ke=1e7,rigid_joint_angular_ke=1e5)
solver.joint_rod_rest_kb_local.zero_();solver.joint_rod_rest_twist.zero_()
s0=model.state();s1=model.state();ctrl=model.control();contacts=pipeline.contacts();dt=1/3840
robot_prev=wp.array(initial_robot,dtype=wp.transform);robot_next=wp.array(initial_robot,dtype=wp.transform);substep=wp.zeros(1,dtype=wp.int32)
@wp.kernel
def drive(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),previous:wp.array(dtype=wp.transform),next:wp.array(dtype=wp.transform),counter:wp.array(dtype=wp.int32),offset:int):
 j=wp.tid();u=float(counter[0]+1)/64.;a=previous[j];bb=next[j];pa=wp.transform_get_translation(a);pb=wp.transform_get_translation(bb);ra=wp.transform_get_rotation(a);rb=wp.transform_get_rotation(bb)
 q[offset+j]=wp.transform(pa+(pb-pa)*u,wp.quat_slerp(ra,rb,u));e=rb*wp.quat_inverse(ra)
 if e[3]<0.:e=-e
 qd[offset+j]=wp.spatial_vector((pb-pa)*60.,wp.vec3(e[0],e[1],e[2])*120.)
@wp.kernel
def preload(force:wp.array(dtype=wp.spatial_vector),ids:wp.array(dtype=wp.int32),torques:wp.array(dtype=wp.vec3)):
 j=wp.tid();force[ids[j]]+=wp.spatial_vector(wp.vec3(0.),torques[j])
@wp.kernel
def tick(counter:wp.array(dtype=wp.int32)):counter[0]+=1
topids=wp.array(tops,dtype=wp.int32);torques=wp.array([Rotation.from_euler('z',a).apply([0,.03,0]) for a in CLIP_YAWS],dtype=wp.vec3)
with wp.ScopedCapture() as cap:
 for _ in range(64):
  s0.clear_forces();wp.launch(preload,3,inputs=[s0.body_f,topids,torques]);wp.launch(drive,len(robot_ids),inputs=[s0.body_q,s0.body_qd,robot_prev,robot_next,substep,robot_start]);pipeline.collide(s0,contacts);solver.step(s0,s1,ctrl,contacts,dt);wp.launch(tick,1,inputs=[substep]);s0,s1=s1,s0
from scipy.spatial.transform import Slerp
from retention import lid_ceiling
ids=np.array(rod_ids).reshape(6,N)
def retention(q):
 result=[]
 for origin,yaw,top in zip(CLIP_ORIGINS,CLIP_YAWS,tops):
  rot=Rotation.from_euler('z',yaw);mask=[]
  for chain in ids:
   qq=q[chain];v=Rotation.from_quat(qq[:,3:]).apply(np.tile([0,0,DS/2],(N,1)))
   aa=rot.inv().apply(qq[:,:3]-v-origin);bb=rot.inv().apply(qq[:,:3]+v-origin);options=[]
   for a,z in zip(aa,bb):
    if (a[1]+.015)*(z[1]+.015)<=0 and abs(z[1]-a[1])>1e-10:options.append(a+(z-a)*(-.015-a[1])/(z[1]-a[1]))
   if not options:mask.append(False);continue
   pt=min(options,key=lambda x:abs(x[0]-.018));world=origin+rot.apply(pt);lp=Rotation.from_quat(q[top,3:]).inv().apply(world-q[top,:3]);roof=lid_ceiling(np.array([lp[0]]))[0]
   mask.append(bool(.005<pt[0]<.028 and pt[2]>.0118 and np.isfinite(roof) and roof-lp[2]>.0008))
  result.append(mask)
 return result

def mixrot(a,b,u):return Slerp([0,1],Rotation.from_quat([a.as_quat(),b.as_quat()]))([u])[0]
def smooth(u):u=float(np.clip(u,0,1));return u*u*(3-2*u)
def interpolate(keys,t):
 for aa,bb in zip(keys[:-1],keys[1:]):
  u=smooth((t-aa[0])/(bb[0]-aa[0]));pos=(1-u)*aa[1]+u*bb[1];rot=mixrot(aa[2],bb[2],u);width=(1-u)*aa[3]+u*bb[3]
  if t<=bb[0]:return pos,rot,width
 return keys[-1][1:]

poses=[];jhistory=[];widths=[];targets=[];rotations=[];phases=[];retained=[];latch_history=[];events=[];counts=[];elapsed_history=[];attempt_history=[]
start=time.time();lastrobot=initial_robot;center=home.copy();toolrot=TOOL_ROT;width=.06;lastphase=None;keys=None;grasp_ids=None
latch_state=[False,False];clip_index=-1;grasp_offset=np.array([0,-.005,.014]);clip_goal=None
# Two 15-second connector sequences, then three 14-second clip sequences.
start_frame=0
if args.resume:
 saved=np.load(ROOT/args.resume);assert 'state_velocity' in saved
 start_frame=len(saved['poses'])
 recovery_start=args.recovery_start if args.recovery_start is not None else start_frame/60
 allowed=([round((recovery_start+v)*60) for v in [0,3,17,31,45]] if args.recover_home else [round((args.retry_start+v)*60) for v in [0,15,30,44,58,72]] if args.retry_first else [990,1890,2730,3570,4410])
 assert args.repair_clips or start_frame in allowed, 'Resume only from a supported saved state'
 if args.repair_clips:assert all(saved['latch_history'][-1]) and float(saved['jaw_width'][-1])>=.06, 'Repair starts after release with both plugs locked'
 s0.body_q.assign(saved['poses'][-1]);s1.body_q.assign(saved['poses'][-1]);s0.body_qd.assign(saved['state_velocity']);s1.body_qd.assign(saved['state_velocity'])
 poses=list(saved['poses']);jhistory=list(saved['joint_positions']);widths=list(saved['jaw_width']);targets=list(saved['tcp_targets']);rotations=list(saved['tool_rotations']);phases=list(saved['phases']);latch_history=list(saved['latch_history']);counts=list(saved['contact_counts']) if 'contact_counts' in saved else [-1]*start_frame
 elapsed_history=list(saved['phase_elapsed']) if 'phase_elapsed' in saved else [max(0,i/60-1.5) for i in range(start_frame)];attempt_history=list(saved['attempt_history']) if 'attempt_history' in saved else [1]*start_frame
 qj=jhistory[-1].copy();center=targets[-1].copy();width=float(widths[-1]);toolrot=Rotation.from_quat(rotations[-1]);lastrobot=saved['poses'][-1,robot_ids].copy();robot_prev.assign(lastrobot);robot_next.assign(lastrobot)
 latch_state=list(map(bool,latch_history[-1]));enabled[latches]=latch_state;model.joint_enabled.assign(enabled)
 if args.retry_first and abs(start_frame/60-args.retry_start)<1e-6:assert not latch_state[0], 'First connector is already latched; do not retry its grasp'
 if args.recover_home:
  assert all(latch_state), 'Both connectors must already be locked'
  unwind_center=center.copy();unwind_width=width;unwind_yaw=(toolrot*TOOL_ROT.inv()).as_euler('xyz')[2]
  if unwind_yaw>0:unwind_yaw-=2*np.pi
  unwind_residual=Rotation.from_euler('z',unwind_yaw).inv()*toolrot*TOOL_ROT.inv()
  if start_frame==round(recovery_start*60):assert width>=.06 and center[2]>.12, 'Unwind only with open fingers in free space'
 events.append(dict(event='restart_from_saved_physical_state',time=start_frame/60,source=args.resume,detail='Positions and velocities preserved; contact-history warm start reset'))
for frame in range(start_frame,round(args.seconds*60)):
 t=frame/60;current=s0.body_q.numpy();phase='settle';local=t
 attempt=1
 if args.repair_clips:
  elapsed_frames=frame-start_frame
  repair_slot=elapsed_frames//840
  if repair_slot<len(args.repair_clips):phase='repair';k=args.repair_clips[repair_slot]-1;local=(elapsed_frames%840)/60
  else:k=-1
 elif args.recover_home:
  assert args.resume
  elapsed_frames=frame-round(recovery_start*60)
  if elapsed_frames<180:phase='wrist_recovery';k=-1;local=elapsed_frames/60
  elif elapsed_frames<2700:phase='clip';k=(elapsed_frames-180)//840;local=(elapsed_frames-180-k*840)/60
  else:k=-1
 elif args.retry_first:
  assert args.resume, 'Retry mode continues a saved first attempt'
  if t<args.retry_start+15:phase='connector';k=0;local=t-args.retry_start;attempt=round(args.retry_start/15)+1
  elif t<args.retry_start+30:phase='connector';k=1;local=t-(args.retry_start+15)
  elif t<args.retry_start+72:phase='clip';k=int((t-args.retry_start-30)//14);local=t-(args.retry_start+30+14*k)
  else:k=-1
 else:
  if 1.5<=t<31.5:phase='connector';k=int((t-1.5)//15);local=t-(1.5+15*k)
  elif 31.5<=t<73.5:phase='clip';k=int((t-31.5)//14);local=t-(31.5+14*k)
  else:k=-1
 marker=(phase,k,attempt)
 if marker!=lastphase:
  if (phase=='connector' and k==1 and not latch_state[0]) or (phase=='clip' and not all(latch_state)):
   print('STOP: previous connector did not latch',flush=True);break
  phase_start=center.copy();phase_rot=toolrot;phase_width=width;keys=None;lastphase=marker;calibrated_lift=False;calibrated_socket=False;servo_offset=np.zeros(3);servo_rotation=Rotation.identity()
  if phase=='clip':
   rot=Rotation.from_euler('z',CLIP_YAWS[k]);target=CLIP_CENTERS[k]+rot.apply([.07,.045,0]);chains=current[ids,:3]
   # Select material positions near this clip, downstream of its lid.
   localq=rot.inv().apply(chains.reshape(-1,3)-CLIP_CENTERS[k]).reshape(6,N,3)
   grasp_ids=np.argmin(abs(localq[:,:,1]-.045),axis=1);clip_goal=None
  if phase=='repair':
   mask=retention(current)[k];missing=np.flatnonzero(np.logical_not(mask));cr=Rotation.from_euler('z',CLIP_YAWS[k]);localq=cr.inv().apply(current[ids,:3].reshape(-1,3)-CLIP_CENTERS[k]).reshape(6,N,3)
   repair_cable=int(missing[0]) if len(missing) else -1
   repair_cables=missing.copy();repair_segments=np.array([np.argmin(abs(localq[c,:,1]+.045)) for c in repair_cables],dtype=int)
   events.append(dict(event='upstream_recovery_selection',time=t,clip=k+1,cables=(repair_cables+1).tolist(),retained_before=mask));print('REPAIR',events[-1],flush=True)
  print('PHASE',t,phase,k,flush=True)
 if phase=='connector':
  body=current[plugs[k]];br=Rotation.from_quat(body[3:]);bp=body[:3];tracked=bp+br.apply(grasp_offset);gr=br*TOOL_ROT
  if local<=2.7:
   if local<=1.3:
    u=smooth(local/1.3);center=(1-u)*phase_start+u*(tracked+[0,0,.080]);toolrot=mixrot(phase_rot,gr,u);width=(1-u)*phase_width+u*.060
   else:
    u=smooth((local-1.3)/1.4);center=tracked+[0,0,.080*(1-u)];toolrot=gr;width=.060
   pickup=center.copy();pickuprot=toolrot;pickupbody=bp.copy();pickupbr=br
  else:
   if keys is None:
    sr=Rotation.from_euler('z',SOCKET_YAWS[k]);tilt=sr*Rotation.from_euler('x',25,degrees=True);angle=math.radians(25);tiltedpos=SOCKET_POSITIONS[k]+sr.apply([0,.0135*(math.cos(angle)-1)+.004*math.sin(angle),.0135*math.sin(angle)+.004*(math.cos(angle)-1)])
    seat=SOCKET_POSITIONS[k]+sr.apply(grasp_offset);tilttcp=tiltedpos+tilt.apply(grasp_offset)
    keys=[(2.7,pickup,pickuprot,.060),(3.6,pickup,pickuprot,.0278),(5.,pickup+np.array([0,0,.075]),pickuprot,.0278),(7.,tilttcp+np.array([0,0,.065]),tilt*TOOL_ROT,.0278),(8.6,tilttcp,tilt*TOOL_ROT,.0278),(10.4,seat,sr*TOOL_ROT,.0278),(11.,seat,sr*TOOL_ROT,.0278),(11.9,seat,sr*TOOL_ROT,.070),(13.2,seat+np.array([0,0,.10]),sr*TOOL_ROT,.070),(15.,home,TOOL_ROT,.060)]
   # Measure actual in-hand pose in free space. Commands still act only
   # through the robot; no object pose or cable force is overwritten.
   if (local>=5.0 and not calibrated_lift) or (local>=7.0 and not calibrated_socket):
    relpos=toolrot.inv().apply(bp-center);relrot=toolrot.inv()*br
    def held_target(pos,rot):
     tr=rot*relrot.inv();return pos-tr.apply(relpos),tr
    stp,strt=held_target(SOCKET_POSITIONS[k],sr);ttp,ttr=held_target(tiltedpos,tilt)
    if local>=7.0:
     keys=[(7.,center.copy(),toolrot,.0278),(8.6,ttp,ttr,.0278),(10.4,stp,strt,.0278),(11.,stp,strt,.0278),(11.9,stp,strt,.070),(13.2,stp+np.array([0,0,.10]),strt,.070),(15.,home,TOOL_ROT,.060)];calibrated_socket=True
    else:
     keys=[(5.,center.copy(),toolrot,.0278),(7.,ttp+np.array([0,0,.065]),ttr,.0278),(8.6,ttp,ttr,.0278),(10.4,stp,strt,.0278),(11.,stp,strt,.0278),(11.9,stp,strt,.070),(13.2,stp+np.array([0,0,.10]),strt,.070),(15.,home,TOOL_ROT,.060)];calibrated_lift=True
    events.append(dict(event='in_hand_pose_calibration',time=t,connector=k+1,attempt=attempt,plug_origin_in_tool_m=relpos.tolist(),plug_rotation_in_tool_xyzw=relrot.as_quat().tolist()));print('CALIBRATE',events[-1],flush=True)
   center,toolrot,width=interpolate(keys,local)
   if 8.6<=local<11.0:
    # Keep the plug's front top corner inside the front wall and its
    # leading lower edge on the floor as it rotates. The old fixed-pivot
    # path intersected the front stop during angled entry.
    angle=math.radians(25)*(1-smooth((local-8.6)/1.8));desired_rotation=sr*Rotation.from_euler('x',angle)
    desired_position=SOCKET_POSITIONS[k]+sr.apply([0,.0135*(math.cos(angle)-1)+.004*math.sin(angle),.0135*math.sin(angle)+.004*(math.cos(angle)-1)])
    center,toolrot=held_target(desired_position,desired_rotation)
    if not latch_state[k]:
     servo_offset+=.12*(desired_position-bp)
     if np.linalg.norm(servo_offset)>.008:servo_offset*=.008/np.linalg.norm(servo_offset)
     error=(desired_rotation*br.inv()).as_rotvec();increment=.12*error
     if np.linalg.norm(increment)>math.radians(.75):increment*=math.radians(.75)/np.linalg.norm(increment)
     servo_rotation=Rotation.from_rotvec(increment)*servo_rotation
     rv=servo_rotation.as_rotvec()
     if np.linalg.norm(rv)>math.radians(20):servo_rotation=Rotation.from_rotvec(rv*math.radians(20)/np.linalg.norm(rv))
   if k==1 and local>=13.2:
    # Explicitly unwind through increasing world yaw. A shortest-quaternion
    # interpolation near 180 degrees can choose the wrong wrist winding.
    u=smooth((local-13.2)/1.8);residual=Rotation.from_euler('z',-math.pi).inv()*strt*TOOL_ROT.inv()
    toolrot=Rotation.from_euler('z',-math.pi*(1-u))*mixrot(residual,Rotation.identity(),u)*TOOL_ROT
   if local>=8.6:
    fade=1-smooth((local-11.9)/3.1);center=center+fade*servo_offset;toolrot=Rotation.from_rotvec(fade*servo_rotation.as_rotvec())*toolrot
   if 10.4<=local<15.0 and not latch_state[k]:
    err=np.linalg.norm(bp-SOCKET_POSITIONS[k]);angle=(Rotation.from_euler('z',SOCKET_YAWS[k]).inv()*br).magnitude()
    if err<.0015 and angle<math.radians(6):
     enabled[latches[k]]=True;model.joint_enabled.assign(enabled);latch_state[k]=True;events.append(dict(time=t,event='connector_latched',connector=k+1,position_error_mm=err*1000,angle_error_deg=math.degrees(angle)));print('LATCH',events[-1],flush=True)
 elif phase=='clip':
  cr=Rotation.from_euler('z',CLIP_YAWS[k]);gr=cr*TOOL_ROT;pts=current[ids[np.arange(6),grasp_ids],:3]
  pp=cr.inv().apply(pts-CLIP_CENTERS[k]);mid=np.array([(pp[:,0].min()+pp[:,0].max())/2,pp[:,1].mean(),pp[:,2].min()-.0002]);tracked=CLIP_CENTERS[k]+cr.apply(mid)
  tracked[2]=max(.0022,tracked[2])
  if local<=2.7:
   if local<=1.3:
    u=smooth(local/1.3);center=(1-u)*phase_start+u*(tracked+[0,0,.075]);toolrot=mixrot(phase_rot,gr,u);width=(1-u)*phase_width+u*.04
   else:
    u=smooth((local-1.3)/1.4);center=tracked+[0,0,.075*(1-u)];toolrot=gr;width=.04
   pickup=center.copy()
  else:
   if keys is None:
    outside=CLIP_CENTERS[k]+cr.apply([.047,.045,.006]);inside=CLIP_CENTERS[k]+cr.apply([-.013,.045,.002]);clip_goal=inside.copy()
    keys=[(2.7,pickup,gr,.040),(3.6,pickup,gr,.00599),(4.6,pickup+np.array([0,0,.024]),gr,.00599),(6.,outside,gr,.00599),(8.2,inside,gr,.00599),(10.,inside,gr,.00599),(10.8,inside,gr,.085),(12.,inside+np.array([0,0,.09]),gr,.085),(14.,home,TOOL_ROT,.060)]
   if 8.2<=local<10.:
    mask=retention(current)[k]
    if not all(mask):clip_goal-=cr.apply([.006/60,0,0])
    for ii in [4,5,6]:keys[ii]=(keys[ii][0],clip_goal.copy(),keys[ii][2],keys[ii][3])
    keys[7]=(12.,clip_goal+[0,0,.09],gr,.085)
   center,toolrot,width=interpolate(keys,local)
 elif phase=='repair':
  cr=Rotation.from_euler('z',CLIP_YAWS[k]);gr=cr*TOOL_ROT
  if repair_cable<0:
   center=home.copy();toolrot=TOOL_ROT;width=.060
  else:
   pts=current[ids[repair_cables,repair_segments],:3];pp=cr.inv().apply(pts-CLIP_CENTERS[k]);mid=np.array([(pp[:,0].min()+pp[:,0].max())/2,pp[:,1].mean(),pp[:,2].min()-.0002]);tracked=CLIP_CENTERS[k]+cr.apply(mid);tracked[2]=max(.0022,tracked[2]);repair_open=float(np.clip(np.ptp(pp[:,0])+.010,.020,.085))
   if local<=2.7:
    if local<=1.3:
     u=smooth(local/1.3);center=(1-u)*phase_start+u*(tracked+[0,0,.075]);toolrot=mixrot(phase_rot,gr,u);width=(1-u)*phase_width+u*repair_open
    else:
     u=smooth((local-1.3)/1.4);center=tracked+[0,0,.075*(1-u)];toolrot=gr;width=repair_open
    pickup=center.copy()
   else:
    if keys is None:
     # The selected strand is approached upstream, where it is above the seated bundle.
     # Physical support lips cradle it; never close the lips through one another.
     inside=CLIP_CENTERS[k]+cr.apply([-.020,-.045,.001]);clip_goal=inside.copy()
     keys=[(2.7,pickup,gr,repair_open),(3.6,pickup,gr,.00599),(4.6,pickup+[0,0,.006],gr,.00599),(8.2,inside,gr,.00599),(10.,inside,gr,.00599),(10.8,inside,gr,.085),(12.,inside+[0,0,.09],gr,.085),(14.,home,TOOL_ROT,.060)]
    if 8.2<=local<10.:
     if not all(retention(current)[k]):clip_goal-=cr.apply([.004/60,0,0])
     for ii in [3,4,5]:keys[ii]=(keys[ii][0],clip_goal.copy(),keys[ii][2],keys[ii][3])
     keys[6]=(12.,clip_goal+[0,0,.09],gr,.085)
    center,toolrot,width=interpolate(keys,local)
 elif phase=='wrist_recovery':
  u=smooth(local/3);center=(1-u)*unwind_center+u*home;width=(1-u)*unwind_width+u*.06
  toolrot=Rotation.from_euler('z',unwind_yaw*(1-u))*mixrot(unwind_residual,Rotation.identity(),u)*TOOL_ROT
 elif phase=='settle':
  center=home.copy();toolrot=TOOL_ROT;width=.060
 theta=theta_for_gap(width);qj=ik(goal_flange(center,theta,toolrot),qj);nextrobot=packed(robot_poses(qj,theta));robot_prev.assign(lastrobot);robot_next.assign(nextrobot);substep.zero_();wp.capture_launch(cap.graph);lastrobot=nextrobot
 q=s0.body_q.numpy().copy();assert np.isfinite(q).all();poses.append(q);jhistory.append(qj.copy());widths.append(width);targets.append(center.copy());rotations.append(toolrot.as_quat());phases.append(f'{phase}_{k+1}');elapsed_history.append(local);attempt_history.append(attempt);latch_history.append(latch_state.copy());counts.append(int(contacts.rigid_contact_count.numpy()[0]))
 if frame%15==0:retained.append([t,retention(q)])
 if frame%60==0 or frame+1 in ([round((recovery_start+v)*60) for v in [0,3,17,31,45]] if args.recover_home else [round((args.retry_start+v)*60) for v in [0,15,30,44,58,72]] if args.retry_first else [990,1890,2730,3570,4410]):
  np.savez_compressed(ROOT/(args.output+'_checkpoint.npz'),state_velocity=s0.body_qd.numpy(),poses=poses,fps=60,segments=N,segment_length=DS,clip_top_bodies=tops,robot_start=robot_start,robot_names=robot_names,plug_bodies=plugs,plug_anchors=plug_anchors,anchor_q=anchor_q,joint_positions=jhistory,jaw_width=widths,tcp_targets=targets,tool_rotations=rotations,rod_ids=rod_ids,phases=phases,phase_elapsed=elapsed_history,attempt_history=attempt_history,latch_history=latch_history)
  if frame+1 in ([round((recovery_start+v)*60) for v in [0,3,17,31,45]] if args.recover_home else [round((args.retry_start+v)*60) for v in [0,15,30,44,58,72]] if args.retry_first else [990,1890,2730,3570,4410]):
   import shutil
   shutil.copy2(ROOT/(args.output+'_checkpoint.npz'),ROOT/(args.output+f'_boundary_{frame+1}.npz'))
  print('FRAME',frame,'time',round(t,2),'retained',np.sum(retention(q),axis=1).tolist(),'plugs',q[plugs,:3].round(4).tolist(),'latched',latch_state,'wall',round(time.time()-start,1),flush=True)
np.savez_compressed(ROOT/(args.output+'.npz'),state_velocity=s0.body_qd.numpy(),poses=poses,fps=60,segments=N,segment_length=DS,clip_top_bodies=tops,robot_start=robot_start,robot_names=robot_names,plug_bodies=plugs,plug_anchors=plug_anchors,anchor_q=anchor_q,joint_positions=jhistory,jaw_width=widths,tcp_targets=targets,tool_rotations=rotations,rod_ids=rod_ids,phases=phases,phase_elapsed=elapsed_history,attempt_history=attempt_history,latch_history=latch_history,contact_counts=counts)
report=dict(cable_count=6,length_mm=L*1000,diameter_mm=2,segments=N,solver='Newton SolverVBD / compliant ALM',iterations=args.iterations,physics_hz=3840,EI_Nm2=EI,grasp='Contact only; no robot attachments or direct cable forces',connector_modification='Raised T-shaped handling rib for pad clearance over receiver walls',latches='Enabled only within 1.5 mm and 6 degrees of seated pose; models a locked connector',events=events,repair_clips=args.repair_clips,recover_home=args.recover_home,recovery_start=recovery_start if args.recover_home else None,retry_first=args.retry_first,retry_start=args.retry_start,connector_pose_servo="12 percent pose-error integration per 60 Hz control frame; correction limited to 8 mm and 20 degrees; contact-only actuation",resume=args.resume,retention=retained,final_latched=latch_state,seconds=len(poses)/60,runtime_seconds=time.time()-start)
(ROOT/(args.output+'.json')).write_text(json.dumps(report,indent=2)+'\n');print('DONE',flush=True)
