"""Isaac Sim RTX render of solved Newton poses. No procedural gate animation.
Run after simulate_newton.py. This uses the exact mesh scene and its materials.
"""
from pathlib import Path
import argparse
p=argparse.ArgumentParser();p.add_argument('--motion',default='actual_motion');p.add_argument('--stride',type=int,default=2);p.add_argument('--stills-only',action='store_true');p.add_argument('--frames-dir',type=Path,default=Path(__file__).resolve().parent/'work'/'isaac_frames');args=p.parse_args()
ROOT=Path(__file__).parent
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960,'multi_gpu':False,'limit_cpu_threads':8,'renderer':'RayTracedLighting'})
import numpy as np,subprocess
from PIL import Image,ImageDraw,ImageFont
import grip_visuals
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdPhysics,Gf
ctx=omni.usd.get_context();ctx.open_stage(str(ROOT/'actual_clip_scene.usda'))
for _ in range(30):app.update()
stage=ctx.get_stage();d=np.load(ROOT/(args.motion+'.npz'));poses=d['poses'];paths=d['paths'];h=d['history'];fps=int(d['fps'])
# Pure playback: physics is disabled during rendering. Every transform is a
# recorded Newton result, not a hand-authored gate angle.
ops=[]
for path in paths:
    prim=stage.GetPrimAtPath(str(path))
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
    x=UsdGeom.Xformable(prim);x.ClearXformOpOrder();ops.append(x.AddTransformOp(opSuffix='playback'))
grip_bars=grip_visuals.create(stage)
rep.orchestrator.set_capture_on_play(False)
product=rep.create.render_product('/World/Cameras/Closeup',(1280,960))
rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product.path])
frames=args.frames_dir/args.motion;frames.mkdir(parents=True,exist_ok=True)
def pose_frame(i):
    for op,q in zip(ops,poses[i]):
        matrix=Gf.Matrix4d(1);matrix.SetRotate(Gf.Quatd(float(q[6]),Gf.Vec3d(*q[3:6])));matrix.SetTranslateOnly(Gf.Vec3d(*q[:3]));op.Set(matrix)
    grip_visuals.update(grip_bars,poses[i],i/fps)
    rep.orchestrator.step(rt_subframes=4,delta_time=0.,pause_timeline=True)
    data=rgb.get_data()
    if isinstance(data,dict):data=data['data']
    a=np.asarray(data)
    if a.ndim!=3 or a.shape[0]==0:raise RuntimeError('RTX returned no RGB pixels')
    if a[:,:,:3].std()<1:raise RuntimeError('RTX returned a blank frame')
    picture=Image.fromarray(a[:,:,:3])
    draw=ImageDraw.Draw(picture)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
    draw.rectangle((0,0,1280,83),fill=(15,22,30))
    draw.text((22,12),'Newton VBD | Six 2 mm cables | Two grips ~1 inch outside clip',font=font,fill='white')
    phase='Pulling into clip' if i/fps<2.5 else ('Lowering into clip' if i/fps<3.2 else 'Grips released - passive settling')
    draw.text((22,46),f'{phase} | Simulated time: {i/fps:.2f} s | Cyan bars mark grasp points',font=font,fill=(110,215,235))
    return picture
try:
    # Let the RTX product initialize before collecting the first real frame.
    for _ in range(3):rep.orchestrator.step(rt_subframes=4,delta_time=0.)
    keyframes={'actual_before':0,'actual_open':int(np.argmin(h[:,1])),'actual_after':len(poses)-1}
    if not args.stills_only:
        for n,i in enumerate(range(0,len(poses),args.stride)):
            pose_frame(i).save(frames/f'frame_{n:04}.png')
            if n%30==0:print('RENDER',n,i,flush=True)
        subprocess.run(['ffmpeg','-y','-v','error','-framerate',str(fps/args.stride),'-i',str(frames/'frame_%04d.png'),'-frames:v',str(len(range(0,len(poses),args.stride))),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/'actual_assets_isaac.mp4')],check=True)
    for name,i in keyframes.items():pose_frame(i).save(ROOT/(name+'.png'))
    rgb.detach([product.path]);product.destroy()
    product=rep.create.render_product('/World/Cameras/Overview',(1280,960));rgb.attach([product.path])
    for _ in range(3):rep.orchestrator.step(rt_subframes=4,delta_time=0.)
    pose_frame(0).save(ROOT/'assembly_before.png')
    pose_frame(len(poses)-1).save(ROOT/'actual_assembly.png')
    print('RENDER_COMPLETE',flush=True)
finally:
    rgb.detach([product.path]);product.destroy();app.close()
