"""Newton OpenGL preview of recorded Newton poses; no second Isaac/Kit process."""
from pathlib import Path
import argparse,subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
from pxr import Usd,UsdGeom,UsdShade
import warp as wp
import newton,newton.viewer
from cable_config import SEGMENTS,HALF_SEGMENT,GRIP_INDICES
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--stills-only',action='store_true');args=p.parse_args()
wp.init();wp.set_device('cpu')
d=np.load(ROOT/'actual_motion.npz');poses=d['poses'];paths=d['paths'];fps=int(d['fps']);history=d['history']
stage=Usd.Stage.Open(str(ROOT/'actual_clip_scene.usda'));b=newton.ModelBuilder(gravity=(0.,0.,0.))
cfg=newton.ModelBuilder.ShapeConfig(density=0.,has_shape_collision=False,has_particle_collision=False)
qaxis=wp.quat(*Rotation.from_euler('x',-90,degrees=True).as_quat())
def color(prim):
    mat,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    if mat:
        shader=UsdShade.Shader(stage.GetPrimAtPath(str(mat.GetPath())+'/Surface'))
        v=shader.GetInput('diffuseColor').Get()
        if v is not None:return tuple(map(float,v))
    return (.5,.5,.5)
def addmesh(path,body):
    prim=stage.GetPrimAtPath(path);m=UsdGeom.Mesh(prim)
    vertices=np.array(m.GetPointsAttr().Get(),dtype=np.float32);indices=np.array(m.GetFaceVertexIndicesAttr().Get(),dtype=np.int32)
    normals=None
    if path.startswith(('/World/ClipBottom/','/World/ClipTop/')):
        triangles=vertices[indices.reshape(-1,3)]
        face_normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);face_normals/=np.maximum(np.linalg.norm(face_normals,axis=1,keepdims=True),1.e-12)
        vertices=triangles.reshape(-1,3);indices=np.arange(len(vertices),dtype=np.int32);normals=np.repeat(face_normals,3,axis=0)
    b.add_shape_mesh(body=body,mesh=newton.Mesh(vertices,indices,normals=normals,compute_inertia=False),cfg=cfg,color=color(prim))
addmesh('/World/ClipBottom/Visual',-1)
b.add_shape_box(body=-1,xform=wp.transform(wp.vec3(.025,-.015,-.003),wp.quat_identity()),hx=.12,hy=.20,hz=.003,cfg=cfg,color=(.3,.34,.39))
b.add_shape_cylinder(body=-1,xform=wp.transform(wp.vec3(.003,-.015,.022),qaxis),radius=.0018,half_height=.016,cfg=cfg,color=(.55,.57,.60))
for k,path in enumerate(paths):
    body=b.add_link(xform=wp.transform(wp.vec3(*poses[0,k,:3]),wp.quat(*poses[0,k,3:])),is_kinematic=True,label=str(path))
    assert body==k
    if 'Cable_' in str(path):
        prim=stage.GetPrimAtPath(str(path)+'/Shape')
        b.add_shape_capsule(body=body,xform=wp.transform(wp.vec3(0.),qaxis),radius=.001,half_height=HALF_SEGMENT,cfg=cfg,color=color(prim))
    else:addmesh(str(path)+'/Visual',body)
for side in range(2):
    for jaw in range(2):
        body=b.add_link(is_kinematic=True,label=f'Grip_{side}_{jaw}')
        b.add_shape_box(body=body,hx=.0095,hy=.002,hz=.00065,cfg=cfg,color=(.05,.65,.8))
model=b.finalize(device='cpu');state=model.state()
viewer=newton.viewer.ViewerGL(width=1280,height=960,headless=True,enable_cuda_interop=newton.viewer.ViewerGL.CudaInterop(0))
viewer.set_model(model);viewer.camera.near=.0001;viewer.camera.far=10.;viewer.camera.fov=35.;viewer.renderer.draw_sky=False;viewer.renderer.sky_upper=(.88,.90,.93)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
frames=ROOT/'work'/'frames';frames.mkdir(parents=True,exist_ok=True)
def camera(overview=False):
    eye=np.array([.28,-.36,.25] if overview else [.13,-.155,.095]);target=np.array([.025,-.015,.015] if overview else [.012,-.015,.020]);forward=(target-eye)/np.linalg.norm(target-eye)
    viewer.set_camera(wp.vec3(*eye),pitch=float(np.degrees(np.arcsin(forward[2]))),yaw=float(np.degrees(np.arctan2(forward[1],forward[0]))))
def frame(i):
    q=np.zeros((len(paths)+4,7),dtype=np.float32);q[:len(paths)]=poses[i];q[len(paths):,6]=1.
    for side,seg in enumerate(GRIP_INDICES):
        center=np.mean([poses[i,1+c*SEGMENTS+seg,:3] for c in range(6)],axis=0)
        for jaw in range(2):
            pos=center.copy();pos[2]+=(-1 if jaw==0 else 1)*.0023
            if i/fps>=3.2:pos[2]=-1.
            q[len(paths)+side*2+jaw,:3]=pos
    state.body_q.assign(q)
    viewer.begin_frame(i/fps);viewer.log_state(state);viewer.end_frame()
    pixels=viewer.get_frame().numpy();picture=Image.fromarray(pixels)
    draw=ImageDraw.Draw(picture);draw.rectangle((0,0,1280,84),fill=(15,22,30))
    draw.text((22,12),'Newton VBD | Six 2 mm x 12 inch cables | Two-grip pull',font=font,fill='white')
    phase='Pulling into clip' if i/fps<2.5 else ('Lowering into clip' if i/fps<3.2 else 'Grips released - passive settling')
    draw.text((22,46),f'{phase} | Simulated time: {i/fps:.2f} s | Newton OpenGL replay',font=font,fill=(110,215,235))
    return picture
try:
    camera(True)
    frame(0)
    if not args.stills_only:
        for n,i in enumerate(range(0,len(poses),2)):
            frame(i).save(frames/f'frame_{n:04}.png')
            if n%30==0:print('RENDER',n,i,flush=True)
        subprocess.run(['ffmpeg','-y','-v','error','-framerate','30','-i',str(frames/'frame_%04d.png'),'-frames:v',str(len(range(0,len(poses),2))),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/'actual_assets_newton.mp4')],check=True)
    for name,i in [('actual_before',0),('actual_open',int(np.argmin(history[:,1]))),('actual_after',len(poses)-1)]:frame(i).save(ROOT/(name+'.png'))
    camera(True);frame(0).save(ROOT/'assembly_before.png');frame(len(poses)-1).save(ROOT/'actual_assembly.png')
    print('RENDER_COMPLETE',flush=True)
finally:viewer.close()
