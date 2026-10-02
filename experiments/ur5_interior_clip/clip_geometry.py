"""Original clip mesh geometry. No scaling; translate onto the shared table."""
from pathlib import Path
import json
import numpy as np
from pxr import Usd,UsdGeom
ROOT=Path(__file__).resolve().parent
CLIP_ORIGIN=np.array([-.018,.215,.001])
GEOMETRY=json.loads((ROOT/'geometry_check.json').read_text())
HOLE_BOTTOM=np.array(GEOMETRY['bottom_hole_local_mm'])*.001
HOLE_TOP=np.array(GEOMETRY['top_hole_local_mm'])*.001

def clip_meshes():
 stage=Usd.Stage.Open(str(ROOT/'actual_clip_scene.usda'))
 def read(path):
  m=UsdGeom.Mesh(stage.GetPrimAtPath(path));v=np.array(m.GetPointsAttr().Get(),dtype=np.float32);f=np.array(m.GetFaceVertexIndicesAttr().Get(),dtype=np.int32)
  a=m.GetPrim().GetAttribute('xformOp:scale')
  if a and a.Get() is not None:v*=np.array(a.Get(),dtype=np.float32)
  return v,f
 return {'bottom':read('/World/ClipBottom/Visual'),'support':read('/World/ClipBottom/EntrySupport'),'top':read('/World/ClipTop/Visual'),'convex_top':[read(str(p.GetPath())) for p in stage.GetPrimAtPath('/World/ClipTop/Collision').GetChildren()]}
