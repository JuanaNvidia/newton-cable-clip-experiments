"""Unified Newton 1.6 VBD/AVBD simulation; Isaac Sim is used only for rendering.
Run in an environment with newton==1.6.0 and warp-lang==1.17.0.
"""
from pathlib import Path
import argparse,json,math,time
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom
import newton
import warp as wp
from cable_config import *
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=5);p.add_argument('--substeps',type=int,default=64);p.add_argument('--iterations',type=int,default=40);p.add_argument('--output',default='actual_motion');p.add_argument('--device',default='cuda:0');p.add_argument('--no-gate-contact',action='store_true');p.add_argument('--checkpoint',type=Path);args=p.parse_args()
wp.init();wp.set_device(args.device)

@wp.func
def smooth(x:float):
    a=wp.clamp(x,0.0,1.0)
    return a*a*(3.0-2.0*a)

@wp.kernel
def grasp(q:wp.array(dtype=wp.transform),qd:wp.array(dtype=wp.spatial_vector),ids:wp.array(dtype=wp.int32),initial:wp.array(dtype=wp.vec3),clock:wp.array(dtype=float),forces:wp.array(dtype=wp.spatial_vector),final_x:float):
    j=wp.tid();t=clock[0]
    if t<3.2:
        body=ids[j];pos=wp.transform_get_translation(q[body]);v=wp.spatial_top(qd[body])
        target=initial[j];target[0]-=(0.055-final_x)*smooth((t-0.5)/2.0)
        side=float((j%2)*2-1)
        target[1]+=side*0.004*smooth(t/0.4)
        target[2]+=0.006*smooth((t-0.5)/0.6)*(1.0-smooth((t-2.5)/0.5))
        f=120.0*(target-pos)-0.026666666666666665*v
        f=wp.vec3(wp.clamp(f[0],-0.5,0.5),wp.clamp(f[1],-2.0,2.0),wp.clamp(f[2],-0.5,0.5))*(1.0-smooth((t-3.0)/0.2))
        forces[body]=wp.spatial_vector(f,wp.vec3(0.0))

@wp.kernel
def advance(clock:wp.array(dtype=float),dt:float):
    clock[0]+=dt

# All dimensions are in metres. Source USD provides exact convex prism meshes.
stage=Usd.Stage.Open(str(ROOT/'actual_clip_scene.usda'))
b=newton.ModelBuilder(gravity=(0.,0.,-9.81))
shape_cfg=newton.ModelBuilder.ShapeConfig(density=1000.,ke=1.e6,kd=100.,mu=.5,gap=.0002,margin=0.)
cable_shape_cfg=newton.ModelBuilder.ShapeConfig(**{**vars(shape_cfg),'density':CABLE_DENSITY})
static_cfg=newton.ModelBuilder.ShapeConfig(density=0.,ke=1.e6,kd=100.,mu=.5,gap=.0002,margin=0.)
def mesh_data(path):
    m=UsdGeom.Mesh(stage.GetPrimAtPath(path));v=np.array(m.GetPointsAttr().Get(),dtype=np.float32);f=np.array(m.GetFaceVertexIndicesAttr().Get(),dtype=np.int32)
    scale=m.GetPrim().GetAttribute('xformOp:scale')
    if scale and scale.Get() is not None:v*=np.array(scale.Get(),dtype=np.float32)
    return v,f

def add_mesh(path,body,cfg,convex=False):
    v,f=mesh_data(path);m=newton.Mesh(v,f)
    if convex:m=m.compute_convex_hull()
    return b.add_shape_mesh(body=body,mesh=m,cfg=cfg,label=path)
add_mesh('/World/ClipBottom/Visual',-1,static_cfg)
add_mesh('/World/ClipBottom/EntrySupport',-1,static_cfg,True)
b.add_shape_box(body=-1,xform=wp.transform(wp.vec3(.025,-.015,-.003),wp.quat_identity()),hx=.12,hy=.20,hz=.003,cfg=static_cfg,label='Board')
geometry=json.loads((ROOT/'geometry_check.json').read_text())
hole0=np.array(geometry['bottom_hole_local_mm'])*.001;hole1=np.array(geometry['top_hole_local_mm'])*.001
offset=hole0-hole1
top=b.add_link(xform=wp.transform(wp.vec3(*offset),wp.quat_identity()),label='ClipTop')
top_shapes=[]
for prim in stage.GetPrimAtPath('/World/ClipTop/Collision').GetChildren():top_shapes.append(add_mesh(str(prim.GetPath()),top,shape_cfg,True))
DEG_PER_RAD=180/math.pi
hinge=b.add_joint_revolute(parent=-1,child=top,parent_xform=wp.transform(wp.vec3(*hole0),wp.quat_identity()),child_xform=wp.transform(wp.vec3(*hole1),wp.quat_identity()),axis=wp.vec3(0.,1.,0.),target_pos=0.,target_ke=.0002*DEG_PER_RAD,target_kd=.000005*DEG_PER_RAD,limit_lower=math.radians(-75),limit_upper=0.,collision_filter_parent=False,label='SpringRevoluteHinge')
b.add_articulation([hinge],label='ClipHinge')
qrod=Rotation.from_euler('x',-90,degrees=True).as_quat();rod_quat=wp.quat(*qrod)
rods=[];rod_shapes=[];grip_ids=[];initial=[]
for c,dx in enumerate(OFFSETS):
    chain=[];joints=[]
    for i in range(SEGMENTS):
        pos=[.055+dx,START_Y+(i+.5)*SEGMENT_LENGTH,.014]
        body=b.add_link(xform=wp.transform(wp.vec3(*pos),rod_quat),label=f'Cable_{c}_{i:02}')
        chain.append(body);rods.append(body)
        rod_shapes.append(b.add_shape_capsule(body=body,radius=RADIUS,half_height=HALF_SEGMENT,cfg=cable_shape_cfg))
        if i:
            joints.append(b.add_joint_rod(parent=chain[-2],child=body,parent_xform=wp.transform(wp.vec3(0.,0.,HALF_SEGMENT),wp.quat_identity()),child_xform=wp.transform(wp.vec3(0.,0.,-HALF_SEGMENT),wp.quat_identity()),stretch_stiffness=1.e6,stretch_damping=.1,bend_stiffness=BEND_STIFFNESS,bend_damping=BEND_DAMPING,label=f'CableRod_{c}_{i:02}'))
        if i in GRIP_INDICES:grip_ids.append(body);initial.append(pos)
    b.add_articulation(joints,label=f'Cable_{c}')
connectors=[]
for end in range(2):
    sign=-1 if end==0 else 1
    rot=np.column_stack([[0,sign,0],[0,0,1],[sign,0,0]]);qconn=Rotation.from_matrix(rot).as_quat()
    v,f=mesh_data(f'/World/Connector_{end}/Hull');hull=trimesh.Trimesh(v,f.reshape(-1,3),process=False)
    cfg=newton.ModelBuilder.ShapeConfig(**{**vars(shape_cfg),'density':float(.015/abs(hull.volume))})
    conn=b.add_link(xform=wp.transform(wp.vec3(.055,START_Y if end==0 else END_Y,.014),wp.quat(*qconn)),label=f'Connector_{end}')
    connectors.append(conn);b.add_shape_mesh(body=conn,mesh=newton.Mesh(v,f).compute_convex_hull(),cfg=cfg)
    qr=(Rotation.from_quat(qconn).inv()*Rotation.from_quat(qrod)).as_quat()
    for c,dx in enumerate(OFFSETS):
        local=Rotation.from_quat(qconn).inv().apply([dx,0,0])
        j=b.add_joint_fixed(parent=conn,child=rods[c*SEGMENTS+(0 if end==0 else SEGMENTS-1)],parent_xform=wp.transform(wp.vec3(*local),wp.quat(*qr)),child_xform=wp.transform(wp.vec3(0.,0.,-HALF_SEGMENT if end==0 else HALF_SEGMENT),wp.quat_identity()),label=f'ConnectorJoint_{end}_{c}')
        b.add_articulation([j],label=f'ConnectorAttachment_{end}_{c}')
if args.no_gate_contact:
    for s in top_shapes:
        for c in rod_shapes:b.shape_collision_filter_pairs.append((min(s,c),max(s,c)))
b.color(balance_colors=False)
model=b.finalize(device=args.device)
# Newton's absolute inertia floor inflates millimetre-scale capsule inertias.
# Validate the analytic tensors relatively, then restore them before solver creation.
analytic_inertia=np.array(b.body_inertia,dtype=np.float64).reshape(-1,3,3)
eigenvalues=np.linalg.eigvalsh(analytic_inertia)
assert np.all(eigenvalues>0) and np.all(eigenvalues[:,0]+eigenvalues[:,1]>=eigenvalues[:,2]*(1-1.e-6))
model.body_inertia.assign(analytic_inertia.astype(np.float32))
model.body_inv_inertia.assign(np.linalg.inv(analytic_inertia).astype(np.float32))
print('ANALYTIC_INERTIA_RANGE',float(eigenvalues.min()),float(eigenvalues.max()),flush=True)
model.set_gravity((0.,0.,-9.81))
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=100000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=args.iterations,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=512,friction_epsilon=1.e-4,rigid_joint_linear_ke=1.e7,rigid_joint_angular_ke=1.e5,rigid_joint_linear_k_start=1.e5,rigid_contact_k_start=1.e4)
s0=model.state();s1=model.state();control=model.control();contacts=pipeline.contacts();clock=wp.zeros(1,dtype=float);ids=wp.array(grip_ids,dtype=wp.int32);initial_wp=wp.array(initial,dtype=wp.vec3)
fps=60;dt=1/fps/args.substeps
print('MODEL',model.body_count,'bodies',model.shape_count,'shapes',model.joint_count,'joints',flush=True)
print('TOP_MASS',model.body_mass.numpy()[top],flush=True)

def step_frame():
    global s0,s1
    for _ in range(args.substeps):
        s0.clear_forces()
        wp.launch(grasp,len(grip_ids),inputs=[s0.body_q,s0.body_qd,ids,initial_wp,clock,s0.body_f,FINAL_X])
        pipeline.collide(s0,contacts)
        solver.step(s0,s1,control,contacts,dt)
        wp.launch(advance,1,inputs=[clock,dt])
        s0,s1=s1,s0
print('CAPTURE_START',flush=True)
with wp.ScopedCapture(device=args.device) as capture:step_frame()
graph=capture.graph
print('CAPTURE_READY',flush=True)
paths=['/World/ClipTop']+[f'/World/Cable_{c}/seg_{i:02}' for c in range(COUNT) for i in range(SEGMENTS)]+['/World/Connector_0','/World/Connector_1']
record_ids=[top]+rods+connectors
poses=[];history=[];counts=[];started=time.time()
for frame in range(int(args.seconds*fps)):
    wp.capture_launch(graph)
    q=s0.body_q.numpy()[record_ids].astype(float)
    if not np.isfinite(q).all():raise RuntimeError('Non-finite Newton state')
    # The visual capsules use local Y; Newton rods use local Z.
    q[1:1+COUNT*SEGMENTS,3:]=(Rotation.from_quat(q[1:1+COUNT*SEGMENTS,3:])*Rotation.from_quat(qrod).inv()).as_quat()
    hp=q[0,:3]+Rotation.from_quat(q[0,3:]).apply(hole1)
    drift=np.linalg.norm(hp-hole0);angle=math.degrees(2*math.atan2(q[0,4],q[0,6]))
    middle=q[1+(COUNT//2)*SEGMENTS+SEGMENTS//2,:3]
    poses.append(q);history.append([frame/fps,angle,drift,*middle]);counts.append(int(contacts.rigid_contact_count.numpy()[0]))
    if frame%30==0:print(f'MEASURE t={frame/fps:.2f} angle={angle:.3f} pin_mm={drift*1000:.4f} middle_mm={middle*1000} contacts={counts[-1]} wall={time.time()-started:.1f}',flush=True)
    if args.checkpoint and frame%60==0:
        temp=args.checkpoint.with_suffix('.tmp.npz');np.savez_compressed(temp,poses=poses,history=history,paths=paths,fps=fps,substeps=args.substeps);temp.replace(args.checkpoint)
np.savez_compressed(ROOT/(args.output+'.npz'),poses=poses,history=history,paths=paths,fps=fps,substeps=args.substeps,contact_counts=counts)
h=np.array(history)
result={'engine':'Newton','newton_version':newton.__version__,'warp_version':wp.__version__,'solver':'SolverVBD (VBD/AVBD, compliant ALM)','device':args.device,'iterations':args.iterations,'inertia_policy':'Positive-definite geometry-derived inertia tensors restored after Newton absolute-floor correction','physics_hz':fps*args.substeps,'cable_model':'61 capsule segments per cable joined by native Newton ROD constraints (stretch/shear/bend/twist)','cable_count':COUNT,'cable_diameter_mm':RADIUS*2000,'connector_pitch_mm':PITCH*1000,'clip_scale':1.,'hinge_model':'native Newton REVOLUTE, Y axis, zero-target spring, -75 to 0 degree limits','hinge_stiffness_Nm_per_rad':.0002*DEG_PER_RAD,'hinge_damping_Nms_per_rad':.000005*DEG_PER_RAD,'cable_bend_stiffness_Nm_per_rad':BEND_STIFFNESS,'cable_bend_damping_Nms_per_rad':BEND_DAMPING,'cable_stretch_stiffness_N_per_m':1.e6,'target_bundle_center_x_mm':FINAL_X*1000,'approach_lift_m':.006,'grip_segments':GRIP_INDICES,'grip_distance_outside_clip_mm':abs(START_Y+(GRIP_INDICES[0]+.5)*SEGMENT_LENGTH+.030)*1000,'length_mm':LENGTH*1000,'bend_rigidity_Nm2':BEND_RIGIDITY,'cable_density_kg_m3':CABLE_DENSITY,'release_seconds':3.2,'no_gate_contact':args.no_gate_contact,'min_angle_deg':float(h[:,1].min()),'final_angle_deg':float(h[-1,1]),'max_hinge_error_mm':float(h[:,2].max()*1000),'max_contact_count':max(counts),'elapsed_simulation_wall_seconds':time.time()-started}
(ROOT/(args.output+'.json')).write_text(json.dumps(result,indent=2)+'\n');print('RESULT',json.dumps(result),flush=True)
