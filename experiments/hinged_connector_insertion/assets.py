"""Parametric visual/collision surrogates inferred from the reference footage.
All dimensions are metres; visible proportions are approximate, not measured CAD.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
PIVOT=np.array([0.,-.018,.024]);OPEN_DEG=25.;SEAT=np.array([0.,.018,.0065])
GRAY=[.72,.75,.77];BLACK=[.035,.045,.052];BLUE=[.045,.19,.62];SILVER=[.62,.66,.70]
def box(name,size,pos,color,collision=True):return dict(name=name.replace('-','m'),kind='box',size=size,pos=pos,q=[0,0,0,1],color=color,collision=collision)
def cylinder(name,radius,height,pos,color,q=None,collision=False):return dict(name=name.replace('-','m'),kind='cylinder',size=[radius,height],pos=pos,q=q or [0,0,0,1],color=color,collision=collision)
def plug_parts():
 p=[box('housing',[.042,.027,.008],[0,0,0],GRAY),box('top_ridge',[.043,.002,.002],[0,-.012,.005],GRAY,False),box('rear_ridge',[.043,.002,.002],[0,.012,.005],GRAY,False),box('blue_wire_support',[.020,.006,.007],[0,.009,.0075],BLUE,False)]
 for sign in [-1,1]:
  p += [box(f'end_key_{sign}',[.001,.015,.004],[sign*.0215,0,0],GRAY),box(f'grip_ear_{sign}',[.003,.006,.002],[sign*.022,.0,.005],GRAY,False)]
 for i in range(20):
  x=(i-9.5)*.0018
  p += [box(f'contact_recess_{i:02}',[.0012,.004,.0002],[x,-.003,.00415],[.095,.10,.11],False),box(f'contact_tip_{i:02}',[.00055,.002,.00022],[x,-.003,.0043],[.60,.52,.30],False)]
 return p

def fixture_parts():
 static=[cylinder('mounting_disc',.027,.003,[0,0,-.0005],SILVER,collision=True),cylinder('pedestal',.021,.012,[0,0,.007],BLACK,collision=True)]
 qx=Rotation.from_euler('y',90,degrees=True).as_quat().tolist()
 for sign in [-1,1]:
  static += [cylinder(f'standoff_{sign}',.0026,.016,[sign*.019,.012,.009],SILVER),box(f'bearing_{sign}',[.006,.012,.014],[sign*.027,-.018,.023],BLACK,False),cylinder(f'pivot_head_{sign}',.0035,.002,[sign*.031,-.018,.024],SILVER,qx)]
 static += [box('bearing_support',[.058,.045,.0045],[0,-.003,.01525],BLACK)]
 static += [cylinder('hinge_pin',.002,.063,PIVOT.tolist(),SILVER,qx)]
 moving=[box('cradle_floor',[.050,.040,.005],[0,.018,0],BLACK),box('left_guide',[.003,.035,.010],[-.024,.018,.005],BLACK),box('right_guide',[.003,.035,.010],[.024,.018,.005],BLACK),box('front_stop',[.043,.002,.010],[0,.003,.005],BLACK),box('rear_stop',[.043,.002,.010],[0,.033,.005],BLACK)]
 # Mouth is truly open: no hull fills the cavity. Decorative screws do not collide.
 for sign in [-1,1]:
  moving += [cylinder(f'guide_screw_{sign}',.0025,.001,[sign*.024,.008,.0108],SILVER),box(f'hinge_knuckle_{sign}',[.008,.009,.008],[sign*.016,0,0],BLACK,False)]
 moving += [box('front_latch_face',[.009,.002,.005],[0,-.003,.003],BLACK,False),box('latch_indicator',[.003,.0004,.0018],[0,-.0042,.003],SILVER,False)]
 return static,moving

def mesh(part):
 if part['kind']=='box':m=trimesh.creation.box(extents=part['size'])
 else:m=trimesh.creation.cylinder(radius=part['size'][0],height=part['size'][1],sections=32)
 M=np.eye(4);M[:3,:3]=Rotation.from_quat(part['q']).as_matrix();M[:3,3]=part['pos'];m.apply_transform(M);return m

def usd_parts(stage,path,parts):
 from pxr import UsdGeom,UsdShade,Gf,Sdf
 for part in parts:
  m=mesh(part);prim=UsdGeom.Mesh.Define(stage,path+'/'+part['name']);prim.CreatePointsAttr(m.vertices.tolist());prim.CreateFaceVertexCountsAttr([3]*len(m.faces));prim.CreateFaceVertexIndicesAttr(m.faces.reshape(-1).tolist());prim.CreateSubdivisionSchemeAttr('none');prim.CreateDisplayColorAttr([Gf.Vec3f(*part['color'])])
  matpath='/Looks/M_'+''.join(f'{int(c*255):02x}' for c in part['color']);mat=UsdShade.Material.Define(stage,matpath);shader=UsdShade.Shader.Define(stage,matpath+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*part['color']));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.38);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)
  prim.GetPrim().SetCustomDataByKey('newton_collision',part['collision'])

def build_assets():
 from pxr import Usd,UsdGeom,Gf
 out=ROOT/'assets';out.mkdir(exist_ok=True)
 for filename in ['cable_end_connector','hinged_table_connector']:
  stage=Usd.Stage.CreateNew(str(out/(filename+'.usda')));UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z');root=UsdGeom.Xform.Define(stage,'/Asset');stage.SetDefaultPrim(root.GetPrim())
  if filename=='cable_end_connector':usd_parts(stage,'/Asset/Plug',plug_parts())
  else:
   static,moving=fixture_parts();usd_parts(stage,'/Asset/Base',static);x=UsdGeom.Xform.Define(stage,'/Asset/Receiver');x.AddTranslateOp().Set(Gf.Vec3d(*PIVOT));x.AddRotateXOp().Set(OPEN_DEG);usd_parts(stage,'/Asset/Receiver',moving)
   x.GetPrim().SetCustomDataByKey('hinge_axis','X');x.GetPrim().SetCustomDataByKey('hinge_limits_deg','0 to 45');x.GetPrim().SetCustomDataByKey('physics','Newton native revolute and bistable detent, defined in simulate.py')
  stage.GetRootLayer().Save();stage.GetRootLayer().Export(str(out/(filename+'.usdc')))
 trimesh.util.concatenate([mesh(x) for x in plug_parts()]).export(out/'cable_end_connector.stl')
 static,moving=fixture_parts()
 for name,parts in [('table_connector_base',static),('table_connector_receiver',moving)]:trimesh.util.concatenate([mesh(x) for x in parts]).export(out/(name+'.stl'))
 spec={'units':'metres','plug':plug_parts(),'fixture_static':static,'fixture_receiver':moving,'hinge_pivot':PIVOT.tolist(),'initial_open_degrees':OPEN_DEG,'ideal_seat_in_receiver':SEAT.tolist(),'provenance':'Surrogate modeled from user reference video, approximately 20–24 s and 33–38 s. Dimensions and hidden mechanism assumed.'}
 (out/'geometry.json').write_text(json.dumps(spec,indent=2)+'\n');print('Exported connector and hinged fixture assets')
if __name__=='__main__':build_assets()
