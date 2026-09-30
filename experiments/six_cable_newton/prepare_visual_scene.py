"""Remove the source scene's PhysX physics; retain only geometry/materials/cameras.
The Newton simulation is constructed by simulate_newton.py. This USD is visual-only.
"""
from pathlib import Path
from pxr import Usd,UsdPhysics
r=Path(__file__).resolve().parent
stage=Usd.Stage.Open(str(r/'actual_clip_scene.usda'))
remove=[]
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.Scene):
        remove.append(str(prim.GetPath()));continue
    for api in prim.GetAppliedSchemas():
        if api.startswith(('Physics','Physx')):prim.RemoveAppliedSchema(api)
    for prop in prim.GetProperties():
        if prop.GetName().startswith(('physics:','physx','drive:','limit:')):prim.RemoveProperty(prop.GetName())
for path in remove:stage.RemovePrim(path)
custom=stage.GetRootLayer().customLayerData;custom['physics_backend']='Newton 1.6 SolverVBD; this USD is visual-only. Run simulate_newton.py.';stage.GetRootLayer().customLayerData=custom
stage.GetRootLayer().Save()
print('Prepared visual-only USD; no PhysX simulation remains.')
