"""Isaac-compatible visual USD playback of recorded Newton body poses."""
from pathlib import Path
from scene import TABLE_BOXES,TABLE_LEGS
import numpy as np
from robot import visual_meshes
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf,UsdShade,Sdf,UsdLux
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,clip_meshes
from assets import usd_parts,plug_parts,fixture_parts,box
ROOT=Path(__file__).resolve().parent
d=np.load(ROOT/'motion.npz');poses=d['poses'];N=int(d['segments']);ds=float(d['segment_length']);fps=int(d['fps']);TOP=int(d['clip_top_body']);ROBOT=int(d['robot_start']);robot_visuals=visual_meshes()
stage=Usd.Stage.CreateNew(str(ROOT/'ur5_scene.usda'));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1);world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
sky=UsdLux.DomeLight.Define(stage,'/World/Lighting/Sky');sky.CreateIntensityAttr(800);sky.CreateColorAttr(Gf.Vec3f(.88,.93,1.0))
sun=UsdLux.DistantLight.Define(stage,'/World/Lighting/Sun');sun.CreateIntensityAttr(500);sun.AddRotateXYZOp().Set(Gf.Vec3f(-35,-25,0))
pin=UsdGeom.Cylinder.Define(stage,'/World/HingePin');pin.CreateAxisAttr('Y');pin.CreateRadiusAttr(.0018);pin.CreateHeightAttr(.032);pin.AddTranslateOp().Set(Gf.Vec3d(*map(float,CLIP_ORIGIN+HOLE_BOTTOM)));pin.CreateDisplayColorAttr([Gf.Vec3f(.7,.72,.75)])
static,moving=fixture_parts();usd_parts(stage,'/World/FixtureBase',static)
board=[box('Table_'+str(i),size,center,[.66,.47,.27] if i==0 else [.47,.51,.54]) for i,(center,size) in enumerate(TABLE_BOXES)]
board.append(box('RobotPedestal',[.24,.24,.70],[.50,.52,-.349],[.22,.25,.28]))
for i,(x,y) in enumerate(TABLE_LEGS):board.append(box(f'Leg_{i}',[.024,.024,.70],[x,y,-.36],[.3,.33,.36]))
usd_parts(stage,'/World/Table',board)
paths=['/World/FixedReceiver','/World/CableEndPlug']+[f'/World/Cable_{c}/Segment_{i:02}' for c in range(6) for i in range(N)]+['/World/FreeDistalConnector','/World/SpringClipTop']+[f'/World/Robot/{name}' for name in d['robot_names']]
colors=[(.85,.04,.065),(.95,.06,.08),(.78,.035,.06),(.93,.10,.15),(.035,.19,.8),(.13,.24,.82)]
meshes=clip_meshes()
def clipmesh(path,data,offset=None,color=(.55,.58,.61)):
 v,f=data
 if offset is not None:v=v+offset
 prim=UsdGeom.Mesh.Define(stage,path);prim.CreatePointsAttr(v.tolist());prim.CreateFaceVertexCountsAttr([3]*(len(f)//3));prim.CreateFaceVertexIndicesAttr(f.tolist());prim.CreateSubdivisionSchemeAttr('none');prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
 mat=UsdShade.Material.Define(stage,'/Looks/Material_'+path.replace('/','_'));shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.5);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)
clipmesh('/World/ClipBottom/Visual',meshes['bottom'],CLIP_ORIGIN)
ops=[]
for k,path in enumerate(paths):
 x=UsdGeom.Xform.Define(stage,path);translate=x.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat);orient=x.AddOrientOp(UsdGeom.XformOp.PrecisionFloat);ops.append((translate,orient))
 if k==0:usd_parts(stage,path,moving)
 elif k==1 or k==int(d['distal_body']):usd_parts(stage,path,plug_parts())
 elif k==TOP:clipmesh(path+'/Visual',meshes['top'])
 elif k>=ROBOT:
  for i,(m,color) in enumerate(robot_visuals[k-ROBOT]):clipmesh(path+f'/Mesh_{i}',(np.asarray(m.vertices),np.asarray(m.faces).flatten()),color=tuple(color))
 else:
  cap=UsdGeom.Capsule.Define(stage,path+'/Visual');cap.CreateRadiusAttr(.001);cap.CreateHeightAttr(ds);cap.CreateAxisAttr('Z');cap.CreateDisplayColorAttr([Gf.Vec3f(*colors[(k-2)//N])]);mat=UsdShade.Material.Define(stage,f'/Looks/Wire_{(k-2)//N}');shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colors[(k-2)//N]));mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(mat)
 q=poses[min(90,len(poses)-1),k];translate.Set(Gf.Vec3f(*map(float,q[:3])));orient.Set(Gf.Quatf(float(q[6]),Gf.Vec3f(*map(float,q[3:6]))))
for name,eye,target in [('Overview',(1.3,-1.2,.9),(.24,.25,.18)),('Grasp',(.22,.46,.20),(.03,.245,.028)),('CableClip',(.18,.08,.16),(0,.215,.014)),('OppositeGrasp',(.17,0,.14),(.02,.135,.025)),('OppositePull',(.18,.35,.18),(0,.20,.014))]:
 camera=UsdGeom.Camera.Define(stage,'/World/Cameras/'+name);camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse());camera.CreateClippingRangeAttr(Gf.Vec2f(.0001,10));camera.CreateFocalLengthAttr(30.)
stage.GetRootLayer().customLayerData={'physics':'Newton recorded playback; no active PhysX','asset_status':'Original clipTop/clipBottom meshes; video-inspired approximate connector','retention':'Connector fixed in seated pose; clip has 0.03 Nm closing preload and 10x original spring stiffness', 'robot':'UR5 CB3 kinematic IK, Robotiq four-bar FK, contact-only cable grasp' };stage.GetRootLayer().Save();stage.GetRootLayer().Export(str(ROOT/'ur5_scene.usdc'));stage.GetRootLayer().Export(str(ROOT/'ur5_playback.usdc'))
stage=Usd.Stage.Open(str(ROOT/'ur5_playback.usdc'));stage.SetFramesPerSecond(fps);stage.SetTimeCodesPerSecond(fps);stage.SetStartTimeCode(min(90,len(poses)-1));stage.SetEndTimeCode(len(poses)-1)
for k,path in enumerate(paths):
 translate,orient=UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
 for frame,q in enumerate(poses[:,k]):
  translate.Set(Gf.Vec3f(*map(float,q[:3])),frame);orient.Set(Gf.Quatf(float(q[6]),Gf.Vec3f(*map(float,q[3:6]))),frame)
stage.GetRootLayer().Save();print('Exported visual scene, cameras, and recorded playback.')
