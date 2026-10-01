"""Isaac-compatible visual USD playback of recorded Newton body poses."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf,UsdShade,Sdf
from assets import usd_parts,plug_parts,fixture_parts,box
ROOT=Path(__file__).resolve().parent
d=np.load(ROOT/'motion.npz');poses=d['poses'];N=int(d['segments']);ds=float(d['segment_length']);fps=int(d['fps'])
stage=Usd.Stage.CreateNew(str(ROOT/'connector_scene.usda'));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1);world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
static,moving=fixture_parts();usd_parts(stage,'/World/FixtureBase',static)
board=[box('Wood',[.38,.48,.016],[0,.12,-.010],[.66,.47,.27])]
for i,(center,extent) in enumerate([((-.113,.12,-.0005),(.154,.48,.003)),((.113,.12,-.0005),(.154,.48,.003)),((0,-.078,-.0005),(.072,.084,.003)),((0,.198,-.0005),(.072,.324,.003))]):board.append(box(f'Steel_{i}',list(extent),list(center),[.47,.51,.54]))
usd_parts(stage,'/World/Table',board);far=UsdGeom.Xform.Define(stage,'/World/FarConnector');far.AddTranslateOp().Set(Gf.Vec3d(*d['far_position']));far.AddRotateZOp().Set(180.);usd_parts(stage,'/World/FarConnector',plug_parts())
paths=['/World/FixedReceiver','/World/CableEndPlug']+[f'/World/Cable_{c}/Segment_{i:02}' for c in range(6) for i in range(N)]
colors=[(.85,.04,.065),(.95,.06,.08),(.78,.035,.06),(.93,.10,.15),(.035,.19,.8),(.13,.24,.82)]
ops=[]
for k,path in enumerate(paths):
 x=UsdGeom.Xform.Define(stage,path);op=x.AddTransformOp();ops.append(op)
 if k==0:usd_parts(stage,path,moving)
 elif k==1:usd_parts(stage,path,plug_parts())
 else:
  cap=UsdGeom.Capsule.Define(stage,path+'/Visual');cap.CreateRadiusAttr(.001);cap.CreateHeightAttr(ds);cap.CreateAxisAttr('Z');cap.CreateDisplayColorAttr([Gf.Vec3f(*colors[(k-2)//N])]);mat=UsdShade.Material.Define(stage,f'/Looks/Wire_{(k-2)//N}');shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colors[(k-2)//N]));mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(mat)
 q=poses[0,k];M=Gf.Matrix4d(1);M.SetRotate(Gf.Quatd(float(q[6]),Gf.Vec3d(*map(float,q[3:6]))));M.SetTranslateOnly(Gf.Vec3d(*map(float,q[:3])));op.Set(M)
for name,eye,target in [('Overview',(.31,-.27,.25),(0,.10,.06)),('Insertion',(.115,-.135,.115),(0,0,.044))]:
 camera=UsdGeom.Camera.Define(stage,'/World/Cameras/'+name);camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse());camera.CreateClippingRangeAttr(Gf.Vec2f(.0001,10));camera.CreateFocalLengthAttr(30.)
stage.GetRootLayer().customLayerData={'physics':'Newton recorded playback; no active PhysX','asset_status':'Video-inspired approximate surrogate, not customer CAD'};stage.GetRootLayer().Save();stage.GetRootLayer().Export(str(ROOT/'connector_scene.usdc'));stage.GetRootLayer().Export(str(ROOT/'connector_playback.usdc'))
stage=Usd.Stage.Open(str(ROOT/'connector_playback.usdc'));stage.SetFramesPerSecond(fps);stage.SetTimeCodesPerSecond(fps);stage.SetStartTimeCode(0);stage.SetEndTimeCode(len(poses)-1)
for k,path in enumerate(paths):
 op=UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()[0]
 for frame,q in enumerate(poses[:,k]):
  M=Gf.Matrix4d(1);M.SetRotate(Gf.Quatd(float(q[6]),Gf.Vec3d(*map(float,q[3:6]))));M.SetTranslateOnly(Gf.Vec3d(*map(float,q[:3])));op.Set(M,frame)
stage.GetRootLayer().Save();print('Exported visual scene, cameras, and recorded playback.')
