"""Offscreen Newton OpenGL playback of the solved arch comparison."""
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import warp as wp
import newton,newton.viewer
ROOT=Path(__file__).resolve().parent
wp.init();wp.set_device('cpu');d=np.load(ROOT/'arch_motion.npz');poses=d['poses'];N=int(d['segments']);ds=float(d['segment_length']);fps=int(d['fps'])
b=newton.ModelBuilder(gravity=(0.,0.,0.));cfg=newton.ModelBuilder.ShapeConfig(density=0.,has_shape_collision=False,has_particle_collision=False)
colors=[(.12,.7,.9),(.95,.65,.15),(.9,.18,.22)]
b.add_shape_box(body=-1,hx=.10,hy=.17,hz=.005,xform=wp.transform(wp.vec3(0,0,-.005),wp.quat_identity()),cfg=cfg,color=(.3,.34,.39))
for k,q in enumerate(poses[0]):
    body=b.add_link(xform=wp.transform(wp.vec3(*q[:3]),wp.quat(*q[3:])),is_kinematic=True)
    b.add_shape_capsule(body=body,radius=.001,half_height=ds/2,cfg=cfg,color=colors[k//N])
for c in range(3):
    for sign in [-1,1]:
        b.add_shape_box(body=-1,hx=.008,hy=.008,hz=.007,xform=wp.transform(wp.vec3((c-1)*.045,sign*.12,.007),wp.quat_identity()),cfg=cfg,color=(.55,.6,.65))
        b.add_shape_sphere(body=-1,radius=.003,xform=wp.transform(wp.vec3((c-1)*.045,sign*.12,.015),wp.quat_identity()),cfg=cfg,color=colors[c])
model=b.finalize(device='cpu');state=model.state();viewer=newton.viewer.ViewerGL(width=1280,height=960,headless=True,enable_cuda_interop=newton.viewer.ViewerGL.CudaInterop(0));viewer.set_model(model)
viewer.camera.near=.0001;viewer.camera.far=10.;viewer.camera.fov=35.;viewer.renderer.draw_sky=False;viewer.renderer.sky_upper=(.88,.90,.93)
eye=np.array([.42,-.44,.28]);target=np.array([0,0,.037]);v=(target-eye)/np.linalg.norm(target-eye);viewer.set_camera(wp.vec3(*eye),pitch=float(np.degrees(np.arcsin(v[2]))),yaw=float(np.degrees(np.arctan2(v[1],v[0]))))
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
frames=ROOT/'work'/'arch_frames';frames.mkdir(parents=True,exist_ok=True)
try:
    for n,i in enumerate(range(0,len(poses),2)):
        t=(i+1)/fps;state.body_q.assign(poses[i]);viewer.begin_frame(t);viewer.log_state(state);viewer.end_frame();im=Image.fromarray(viewer.get_frame().numpy());draw=ImageDraw.Draw(im)
        draw.rectangle((0,0,1280,132),fill=(15,22,30));draw.text((22,10),'Newton VBD | 12 inch / 2 mm cable bending comparison',font=font,fill='white')
        draw.text((22,42),'Ends clamped 240 mm apart; free middle; straight rest shape; gravity ON',font=small,fill='white')
        for c,label in enumerate(['0.1x stiffness','Original stiffness','2x stiffness']):draw.text((22+c*405,74),label,font=font,fill=tuple(int(x*255) for x in colors[c]))
        phase='Gravity settling' if t<3 else 'Downward midpoint force: up to 0.02 N' if t<4 else 'Load removed: passive recovery'
        draw.text((22,104),f'{phase} | {t:.2f} s | Solved motion, OpenGL replay',font=small,fill='white')
        im.save(frames/f'frame_{n:04}.png')
        if i==len(poses)-2:im.save(ROOT/'arch_comparison.png')
        if n%60==0:print('RENDER',n,flush=True)
    subprocess.run(['ffmpeg','-y','-v','error','-framerate','30','-i',str(frames/'frame_%04d.png'),'-frames:v',str(len(range(0,len(poses),2))),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/'arch_comparison.mp4')],check=True)
finally:viewer.close()
