"""Thirty-inch harness, three original clips, two fixed receiver sockets."""
import numpy as np
from scipy.spatial.transform import Rotation as R
TABLE_BOXES=[([.025,.39,-.009],[.65,1.02,.020])]
TABLE_LEGS=[(x,y) for x in [-.27,.32] for y in [-.08,.86]]
L=.762;COUNT=6;N=152;DS=L/N;RADIUS=.001;PITCH=.0025
SOCKET_POSITIONS=np.array([[-.025,.015,.0305],[-.025,.760,.0305]])
SOCKET_YAWS=[0.,np.pi]
CLIP_CENTERS=np.array([[-.065,.195,.014],[-.08,.385,.014],[-.065,.575,.014]])
CLIP_YAWS=[np.arctan(.17),0.,-np.arctan(.17)]
CLIP_ORIGINS=np.array([c-R.from_euler('z',a).apply([.018,-.015,.013]) for c,a in zip(CLIP_CENTERS,CLIP_YAWS)])
