"""Isaac-compatible visual USD playback of recorded Newton body poses."""
from pathlib import Path
from motion_io import load_motion,motion_sha256
from scene import TABLE_BOXES,TABLE_LEGS,SOCKET_POSITIONS,SOCKET_YAWS,CLIP_ORIGINS,CLIP_YAWS,CLIP_CENTERS
import numpy as np
from robot import visual_meshes
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,Gf,UsdShade,Sdf,UsdLux
from clip_geometry import CLIP_ORIGIN,HOLE_BOTTOM,clip_meshes
from assets import usd_parts,plug_parts,fixture_parts,box,SEAT
ROOT=Path(__file__).resolve().parent
d=load_motion(ROOT/'motion.npz');poses=d['poses'];N=int(d['segments']);ds=float(d['segment_length']);fps=int(d['fps']);TOPS=list(d['clip_top_bodies']);PLUGS=list(d['plug_bodies']);ROBOT=int(d['robot_start']);robot_visuals=visual_meshes()
stage=Usd.Stage.CreateNew(str(ROOT/'ur5_scene.usda'));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1);world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
sky=UsdLux.DomeLight.Define(stage,'/World/Lighting/Sky');sky.CreateIntensityAttr(800);sky.CreateColorAttr(Gf.Vec3f(.88,.93,1.0))
sun=UsdLux.DistantLight.Define(stage,'/World/Lighting/Sun');sun.CreateIntensityAttr(500);sun.AddRotateXYZOp().Set(Gf.Vec3f(-35,-25,0))
static,moving=fixture_parts()
def fixed_transform(path,pos,rot):
 x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));qq=rot.as_quat();x.AddOrientOp().Set(Gf.Quatf(float(qq[3]),Gf.Vec3f(*map(float,qq[:3]))))
for k,(seat,yaw) in enumerate(zip(SOCKET_POSITIONS,SOCKET_YAWS)):
 rot=Rotation.from_euler('z',yaw);receiver=seat-rot.apply(SEAT);path=f'/World/Socket_{k+1}'
 fixed_transform(path+'/Receiver',receiver,rot);usd_parts(stage,path+'/Receiver',moving)
 fixed_transform(path+'/Base',receiver-rot.apply([0,-.018,.024]),rot);usd_parts(stage,path+'/Base',static)
board=[box('Table_'+str(i),size,center,[.66,.47,.27] if i==0 else [.47,.51,.54]) for i,(center,size) in enumerate(TABLE_BOXES)]
board.append(box('RobotPedestal',[.24,.24,.70],[.50,.52,-.349],[.22,.25,.28]))
for i,(x,y) in enumerate(TABLE_LEGS):board.append(box(f'Leg_{i}',[.024,.024,.70],[x,y,-.36],[.3,.33,.36]))
usd_parts(stage,'/World/Table',board)
paths=['']*poses.shape[1]
for k,body in enumerate(PLUGS):paths[body]=f'/World/Connector_{k+1}'
for c,chain in enumerate(d['rod_ids'].reshape(6,-1)):
 for i,body in enumerate(chain):paths[body]=f'/World/Cable_{c}/Segment_{i:03}'
for k,body in enumerate(TOPS):paths[body]=f'/World/Clip_{k+1}/Top'
for i,name in enumerate(d['robot_names']):paths[ROBOT+i]=f'/World/Robot/{name}'
assert all(paths)
colors=[(.85,.04,.065),(.95,.06,.08),(.78,.035,.06),(.93,.10,.15),(.035,.19,.8),(.13,.24,.82)]
meshes=clip_meshes()
def clipmesh(path,data,offset=None,color=(.55,.58,.61)):
 v,f=data
 if offset is not None:v=v+offset
 prim=UsdGeom.Mesh.Define(stage,path);prim.CreatePointsAttr(v.tolist());prim.CreateFaceVertexCountsAttr([3]*(len(f)//3));prim.CreateFaceVertexIndicesAttr(f.tolist());prim.CreateSubdivisionSchemeAttr('none');prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
 mat=UsdShade.Material.Define(stage,'/Looks/Material_'+path.replace('/','_'));shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.5);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)
for k,(origin,yaw) in enumerate(zip(CLIP_ORIGINS,CLIP_YAWS)):
 path=f'/World/Clip_{k+1}/Bottom';rot=Rotation.from_euler('z',yaw);fixed_transform(path,origin,rot);clipmesh(path+'/Visual',meshes['bottom'])
 pin=UsdGeom.Cylinder.Define(stage,f'/World/Clip_{k+1}/HingePin');pin.CreateAxisAttr('Y');pin.CreateRadiusAttr(.0018);pin.CreateHeightAttr(.032);pin.AddTranslateOp().Set(Gf.Vec3d(*map(float,origin+rot.apply(HOLE_BOTTOM))));qq=rot.as_quat();pin.AddOrientOp().Set(Gf.Quatf(float(qq[3]),Gf.Vec3f(*map(float,qq[:3]))));pin.CreateDisplayColorAttr([Gf.Vec3f(.7,.72,.75)])
ops=[]
for k,path in enumerate(paths):
 x=UsdGeom.Xform.Define(stage,path);translate=x.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat);orient=x.AddOrientOp(UsdGeom.XformOp.PrecisionFloat);ops.append((translate,orient))
 if k in PLUGS:usd_parts(stage,path,plug_parts())
 elif k in TOPS:clipmesh(path+'/Visual',meshes['top'])
 elif k>=ROBOT:
  for i,(m,color) in enumerate(robot_visuals[k-ROBOT]):clipmesh(path+f'/Mesh_{i}',(np.asarray(m.vertices),np.asarray(m.faces).flatten()),color=tuple(color))
 else:
  cap=UsdGeom.Capsule.Define(stage,path+'/Visual');cap.CreateRadiusAttr(.001);cap.CreateHeightAttr(ds);cap.CreateAxisAttr('Z');cap.CreateDisplayColorAttr([Gf.Vec3f(*colors[(k-2)//N])]);mat=UsdShade.Material.Define(stage,f'/Looks/Wire_{(k-2)//N}');shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colors[(k-2)//N]));mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(mat)
 q=poses[0,k];translate.Set(Gf.Vec3f(*map(float,q[:3])));orient.Set(Gf.Quatf(float(q[6]),Gf.Vec3f(*map(float,q[3:6]))))
cameras=[('Overview',(1.24,-.94,1.06),(.06,.36,.07))]
for k,c in enumerate(CLIP_CENTERS):cameras.append((f'Clip{k+1}',c+np.array([.24,-.20,.23]),c))
for k,c in enumerate(SOCKET_POSITIONS):cameras.append((f'Connector{k+1}',c+np.array([.27,-.31,.25]),c))
for name,eye,target in cameras:
 camera=UsdGeom.Camera.Define(stage,'/World/Cameras/'+name);camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,target)),Gf.Vec3d(0,0,1)).GetInverse());camera.CreateClippingRangeAttr(Gf.Vec2f(.0001,10));camera.CreateFocalLengthAttr(30.)
stage.GetRootLayer().customLayerData={'physics':'Newton recorded playback, 10 Hz pose samples with interpolation; no active PhysX','asset_status':'Original clipTop/clipBottom meshes; video-inspired approximate connector','retention':'Two initially dynamic connectors; seat locks gated by alignment. Three original spring clips.', 'robot':'UR5 CB3 kinematic IK, Robotiq four-bar FK, contact-only cable grasp' };stage.GetRootLayer().Save();stage.GetRootLayer().Export(str(ROOT/'ur5_scene.usdc'));stage.GetRootLayer().Export(str(ROOT/'ur5_playback.usdc'))
stage=Usd.Stage.Open(str(ROOT/'ur5_playback.usdc'));stage.SetFramesPerSecond(fps);stage.SetTimeCodesPerSecond(fps);stage.SetStartTimeCode(0);stage.SetEndTimeCode(len(poses)-1)
for k,path in enumerate(paths):
 translate,orient=UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
 for frame in sorted(set(range(0,len(poses),6))|{len(poses)-1}):
  q=poses[frame,k]
  translate.Set(Gf.Vec3f(*map(float,q[:3])),frame);orient.Set(Gf.Quatf(float(q[6]),Gf.Vec3f(*map(float,q[3:6]))),frame)
stage.GetRootLayer().Save();print('Exported visual scene, cameras, and recorded playback.')
