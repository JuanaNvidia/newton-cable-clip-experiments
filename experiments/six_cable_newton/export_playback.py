"""Export an animated USD from the recorded Newton poses, with physics disabled."""
from pathlib import Path
import numpy as np
import grip_visuals
from pxr import Usd,UsdGeom,UsdPhysics,Gf
ROOT=Path(__file__).parent

def main():
    d=np.load(ROOT/'actual_motion.npz');poses=d['poses'];paths=d['paths'];fps=int(d['fps'])
    source=Usd.Stage.Open(str(ROOT/'actual_clip_scene.usda'))
    source.GetRootLayer().Export(str(ROOT/'actual_clip_scene.usdc'))
    source.GetRootLayer().Export(str(ROOT/'actual_clip_playback.usdc'))
    stage=Usd.Stage.Open(str(ROOT/'actual_clip_playback.usdc'))
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
        if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
    stage.SetFramesPerSecond(fps);stage.SetTimeCodesPerSecond(fps);stage.SetStartTimeCode(0);stage.SetEndTimeCode(len(poses)-1)
    for k,path in enumerate(paths):
        xf=UsdGeom.Xformable(stage.GetPrimAtPath(str(path)));xf.ClearXformOpOrder();op=xf.AddTransformOp(opSuffix='recorded')
        for frame,q in enumerate(poses[:,k]):
            mat=Gf.Matrix4d(1);mat.SetRotate(Gf.Quatd(float(q[6]),Gf.Vec3d(*q[3:6])));mat.SetTranslateOnly(Gf.Vec3d(*q[:3]));op.Set(mat,frame)
    bars=grip_visuals.create(stage)
    for frame,pose in enumerate(poses):grip_visuals.update(bars,pose,frame/fps,frame)
    stage.GetRootLayer().Save()
    print('Exported visual scene and recorded USD playback.')
if __name__=='__main__':main()
