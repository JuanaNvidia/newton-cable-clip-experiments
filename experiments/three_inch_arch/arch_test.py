"""Gravity + force deflection test. Straight rest rods; clamped endpoints, free middle."""
from pathlib import Path
import json, math, argparse
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import warp as wp
import newton
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=8);a=p.parse_args()
wp.init();wp.set_device('cuda:0')
N=61;L=.3048;ds=L/N;r=.001;span=.24
# Equal-length chords on a circular arc. This is an initial deformed state,
# not rest curvature: every rod joint has identity local anchor rotations.
angle=brentq(lambda t: ds*math.sin(t/2)/math.sin(t/(2*N))-span,.01,5.)
rad=ds/(2*math.sin(angle/(2*N)))
theta=np.linspace(-angle/2,angle/2,N+1)
y=rad*np.sin(theta);z=.015+rad*(np.cos(theta)-math.cos(angle/2))
b=newton.ModelBuilder(gravity=(0.,0.,-9.81))
# Correct capsule-overlap mass to the solid-cylinder line density.
density=1000*ds/(ds+4*r/3)
cfg=newton.ModelBuilder.ShapeConfig(density=density,ke=1e6,kd=100.,mu=.5,gap=.0002)
b.add_shape_box(body=-1,hx=.10,hy=.19,hz=.005,xform=wp.transform(wp.vec3(0,0,-.005),wp.quat_identity()),cfg=newton.ModelBuilder.ShapeConfig(density=0.))
base_EI=.002*180/math.pi*.005
factors=[.1,1.,2.];chains=[];clamps=[]
for c,factor in enumerate(factors):
    chain=[];joints=[]
    for i in range(N):
        v=np.array([0.,y[i+1]-y[i],z[i+1]-z[i]])/ds
        rot=Rotation.from_euler('x',math.atan2(-v[1],v[2])).as_quat()
        body=b.add_link(xform=wp.transform(wp.vec3((c-1)*.045,(y[i]+y[i+1])/2,(z[i]+z[i+1])/2),wp.quat(*rot)))
        chain.append(body);b.add_shape_capsule(body=body,radius=r,half_height=ds/2,cfg=cfg)
        if i:joints.append(b.add_joint_rod(parent=chain[-2],child=body,parent_xform=wp.transform(wp.vec3(0,0,ds/2),wp.quat_identity()),child_xform=wp.transform(wp.vec3(0,0,-ds/2),wp.quat_identity()),stretch_stiffness=1e6,stretch_damping=.1,bend_stiffness=base_EI*factor/ds,bend_damping=.00572957795*.005/ds))
    b.add_articulation(joints)
    for i,sgn in [(0,-1),(N-1,1)]:
        j=b.add_joint_fixed(parent=-1,child=chain[i],parent_xform=wp.transform(wp.vec3((c-1)*.045,sgn*span/2,.015),wp.quat(*np.asarray(b.body_q[chain[i]])[3:])),child_xform=wp.transform(wp.vec3(0,0,sgn*ds/2),wp.quat_identity()))
        b.add_articulation([j]);clamps.append(j)
    chains.append(chain)
# Each comparison cable is an independent test, with no inter-case contact.
for c in range(3):
    for d in range(c+1,3):
        for i in range(N):
            for j in range(N):b.shape_collision_filter_pairs.append((1+c*N+i,1+d*N+j))
b.color();model=b.finalize(device='cuda:0')
I=np.array(b.body_inertia).reshape(-1,3,3);assert np.linalg.eigvalsh(I).min()>0
model.body_inertia.assign(I.astype(np.float32));model.body_inv_inertia.assign(np.linalg.inv(I).astype(np.float32))
pipeline=newton.CollisionPipeline(model,broad_phase='sap',rigid_contact_max=10000,contact_matching='latest')
solver=newton.solvers.SolverVBD(model,iterations=40,rigid_compliant_alm=True,rigid_contact_history=True,rigid_body_contact_buffer_size=128)
s0=model.state();s1=model.state();ctrl=model.control();contacts=pipeline.contacts();clock=wp.zeros(1,dtype=float);dt=1/3840
clamp_ids=wp.array(clamps,dtype=wp.int32)
@wp.kernel
def move_clamps(frames:wp.array(dtype=wp.transform),ids:wp.array(dtype=wp.int32),clock:wp.array(dtype=float)):
    j=wp.tid();u=wp.clamp((clock[0]-1.0)/4.0,0.0,1.0);u=u*u*(3.0-2.0*u)
    span=0.24+(0.0762-0.24)*u
    old=frames[ids[j]];pos=wp.transform_get_translation(old)
    pos[1]=float((j%2)*2-1)*span/2.0
    frames[ids[j]]=wp.transform(pos,wp.transform_get_rotation(old))
@wp.kernel
def tick(clock:wp.array(dtype=float),dt:float):clock[0]+=dt
with wp.ScopedCapture() as cap:
    for _ in range(64):
        s0.clear_forces();wp.launch(move_clamps,6,inputs=[model.joint_X_p,clamp_ids,clock]);pipeline.collide(s0,contacts);solver.step(s0,s1,ctrl,contacts,dt);wp.launch(tick,1,inputs=[clock,dt]);s0,s1=s1,s0
poses=[];spans=[]
for frame in range(round(a.seconds*60)):
    wp.capture_launch(cap.graph);q=s0.body_q.numpy();assert np.isfinite(q).all();poses.append(q.copy());spans.append(float(model.joint_X_p.numpy()[clamps[1],1]*2))
    if frame%60==0:print('FRAME',frame,'apex_mm',q[np.array(chains)[:,N//2],2]*1000,flush=True)
poses=np.array(poses);np.savez_compressed(ROOT/'arch_motion.npz',poses=poses,fps=60,segments=N,segment_length=ds,factors=factors,spans=spans)
results=[]
for c,factor in enumerate(factors):
    q=poses[:,c*N:(c+1)*N];rot=Rotation.from_quat(q[:,:,3:].reshape(-1,4));vec=rot.apply(np.tile([0,0,ds/2],(len(q)*N,1))).reshape(len(q),N,3)
    ends0=q[:,:,:3]-vec;ends1=q[:,:,:3]+vec
    anchor0=np.column_stack([np.full(len(q),(c-1)*.045),-np.array(spans)/2,np.full(len(q),.015)])
    anchor1=anchor0.copy();anchor1[:,1]*=-1
    results.append({'factor':factor,'EI_Nm2':base_EI*factor,'final_apex_mm':float(q[-1,:,2].max()*1000),'last_second_apex_range_mm':float(np.ptp(q[-60:,:,2].max(axis=1))*1000),'maximum_sideways_excursion_mm':float(np.abs(q[:,:,0]-(c-1)*.045).max()*1000),'max_joint_gap_mm':float(np.linalg.norm(ends1[:,:-1]-ends0[:,1:],axis=-1).max()*1000),'max_end_anchor_error_mm':float(max(np.linalg.norm(ends0[:,0]-anchor0,axis=1).max(),np.linalg.norm(ends1[:,-1]-anchor1,axis=1).max())*1000),'final_actual_endpoint_distance_mm':float(np.linalg.norm(ends1[-1,-1]-ends0[-1,0])*1000)})
report={'length_m':L,'diameter_m':2*r,'initial_endpoint_span_m':span,'final_endpoint_span_m':.0762,'endpoint_elevation_m':.015,'boundary_conditions':'Endpoint clamps translate symmetrically from 240 to 76.2 mm apart between 1 and 5 seconds; orientations stay fixed at initial values. Free middle, straight rest rods, gravity active. Comparison cases cannot collide. No midpoint force.','solver':'Newton 1.6 SolverVBD, 40 iterations, 3840 Hz','results':results}
(ROOT/'arch_results.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
