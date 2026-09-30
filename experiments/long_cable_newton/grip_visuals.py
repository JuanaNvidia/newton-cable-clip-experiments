"""Non-colliding markers for the two virtual six-cable hand grasps.
Forces are applied only in simulate_newton.py; these bars only show grasp sites.
"""
import numpy as np
from pxr import UsdGeom,Gf
from cable_config import COUNT,SEGMENTS,HALF_SEGMENT,GRIP_INDICES
RELEASE_TIME=3.2

def create(stage):
    bars=[]
    for side in range(2):
        group=[]
        for jaw in range(2):
            cube=UsdGeom.Cube.Define(stage,f'/World/VirtualGrips/Grip_{side}/Jaw_{jaw}')
            cube.CreateSizeAttr(1)
            cube.CreateDisplayColorAttr([Gf.Vec3f(.05,.65,.8)])
            move=cube.AddTranslateOp();scale=cube.AddScaleOp()
            group.append((cube,move,scale))
        bars.append(group)
    return bars

def update(bars,pose,time_seconds,time_code=None):
    for side,segment in enumerate(GRIP_INDICES):
        points=np.array([pose[1+c*SEGMENTS+segment,:3] for c in range(COUNT)])
        center=points.mean(axis=0)
        for jaw,(cube,move,scale) in enumerate(bars[side]):
            loc=center.copy();loc[2]+=(-1 if jaw==0 else 1)*.0023
            sz=Gf.Vec3f(float(np.ptp(points[:,0])+.006),.004,.0013)
            visibility='inherited' if time_seconds<RELEASE_TIME else 'invisible'
            if time_code is None:
                move.Set(Gf.Vec3d(*loc));scale.Set(sz);cube.GetVisibilityAttr().Set(visibility)
            else:
                move.Set(Gf.Vec3d(*loc),time_code);scale.Set(sz,time_code);cube.GetVisibilityAttr().Set(visibility,time_code)
