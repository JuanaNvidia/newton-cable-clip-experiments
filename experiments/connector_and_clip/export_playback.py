"""Isaac-compatible visual USD playback of recorded Newton body poses."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf,UsdShade,Sdf
from clip_geometry import CLIP_ORIGIN,clip_meshes
from assets import usd_parts,plug_parts,fixture_parts,box
ROOT=Path(__file__).resolve().parent
d=np.load(ROOT/'motion.npz');poses=d['poses'];N=int(d['segments']);ds=float(d['segment_length']);fps=int(d['fps']);TOP=int(d['clip_top_body'])
stage=Usd.Stage.CreateNew(str(ROOT/'connector_scene.usda'));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1);world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
static,moving=fixture_parts();usd_parts(stage,'/World/FixtureBase',static)
board=[box('Wood',[.38,.68,.016],[0,.22,-.010],[.66,.47,.27])]
for i,(center,extent) in enumerate([((-.113,.22,-.0005),(.154,.68,.003)),((.113,.22,-.0005),(.154,.68,.003)),((0,-.078,-.0005),(.072,.084,.003)),((0,.298,-.0005),(.072,.524,.003))]):board.append(box(f'Steel_{i}',list(extent),list(center),[.47,.51,.54]))
usd_parts(stage,'/World/Table',board);far=UsdGeom.Xform.Define(stage,'/World/FarConnector');far.AddTranslateOp().Set(Gf.Vec3d(*d['far_position']));far.AddRotateZOp().Set(180.);usd_parts(stage,'/World/FarConnector',plug_parts())
paths=['/World/FixedReceiver','/World/CableEndPlug']+[f'/World/Cable_{c}/Segment_{i:02}' for c in range(6) for i in range(N)]+['/World/SpringClipTop']
colors=[(.85,.04,.065),(.95,.06,.08),(.78,.035,.06),(.93,.10,.15),(.035,.19,.8),(.13,.24,.82)]
meshes=clip_meshes()
def clipmesh(path,data,offset=None,color=(.55,.58,.61)):
 v,f=data
 if offset is not None:v=v+offset
 prim=UsdGeom.Mesh.Define(stage,path);prim.CreatePointsAttr(v.tolist());prim.CreateFaceVertexCountsAttr([3]*(len(f)//3));prim.CreateFaceVertexIndicesAttr(f.tolist());prim.CreateSubdivisionSchemeAttr('none');prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
 mat=UsdShade.Material.Define(stage,'/Looks/Clip');shader=UsdShade.Shader.Define(stage,'/Looks/Clip/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.5);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)
clipmesh('/World/ClipBottom/Visual',meshes['bottom'],CLIP_ORIGIN)
ops=[]
for k,path in enumerate(paths):
 x=UsdGeom.Xform.Define(stage,path);translate=x.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat);orient=x.AddOrientOp(UsdGeom.XformOp.PrecisionFloat);ops.append((translate,orient))
 if k==0:usd_parts(stage,path,moving)
 elif k==1:usd_parts(stage,path,plug_parts())
 elif k==TOP:clipmesh(path+'/Visual',meshes['top'])
 else:
  cap=UsdGeom.Capsule.Define(stage,path+'/Visual');cap.CreateRadiusAttr(.001);cap.CreateHeightAttr(ds);cap.CreateAxisAttr('Z');cap.CreateDisplayColorAttr([Gf.Vec3f(*colors[(k-2)//N])]);mat=UsdShade.Material.Define(stage,f'/Looks/Wire_{(k-2)//N}');shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colors[(k-2)//N]));mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(mat)
 q=poses[0,k];translate.Set(Gf.Vec3f(*map(float,q[:3])));orient.Set(Gf.Quatf(float(q[6]),Gf.Vec3f(*map(float,q[3:6]))))
for name,eye,target in [('Overview',(.43,-.34,.34),(0,.18,.105)),('Insertion',(.115,-.135,.115),(0,0,.044)),('CableClip',(.10,.035,.135),(0,.20,.023))]:
 camera=UsdGeom.Camera.Define(stage,'/World/Cameras/'+name);camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse());camera.CreateClippingRangeAttr(Gf.Vec2f(.0001,10));camera.CreateFocalLengthAttr(30.)
stage.GetRootLayer().customLayerData={'physics':'Newton recorded playback; no active PhysX','asset_status':'Original clipTop/clipBottom meshes; video-inspired approximate connector','retention':'Approximate bounded connector retention; clip has 0.03 Nm closing preload and 10x original spring stiffness' };stage.GetRootLayer().Save();stage.GetRootLayer().Export(str(ROOT/'connector_scene.usdc'));stage.GetRootLayer().Export(str(ROOT/'connector_playback.usdc'))
stage=Usd.Stage.Open(str(ROOT/'connector_playback.usdc'));stage.SetFramesPerSecond(fps);stage.SetTimeCodesPerSecond(fps);stage.SetStartTimeCode(0);stage.SetEndTimeCode(len(poses)-1)
for k,path in enumerate(paths):
 translate,orient=UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
 for frame,q in enumerate(poses[:,k]):
  translate.Set(Gf.Vec3f(*map(float,q[:3])),frame);orient.Set(Gf.Quatf(float(q[6]),Gf.Vec3f(*map(float,q[3:6]))),frame)
stage.GetRootLayer().Save();print('Exported visual scene, cameras, and recorded playback.')
