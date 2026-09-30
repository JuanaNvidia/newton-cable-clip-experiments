"""Independent recorded-pose checks against actual clip STL surfaces."""
from pathlib import Path
from cable_config import COUNT,RADIUS,PITCH,OFFSETS,AREA_FACTOR,GRASP_DAMPING,FORCE_CAP,CLIP_SCALE
import json
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
ROOT=Path(__file__).parent

def cross_section(pose,cable):
    rods=pose[1+cable*48:1+(cable+1)*48];rot=Rotation.from_quat(rods[:,3:]);start=rods[:,:3]+rot.apply(np.tile([0,-.0025,0],(48,1)));end=rods[:,:3]+rot.apply(np.tile([0,.0025,0],(48,1)))
    for a,b in zip(start,end):
        if (a[1]+.015)*(b[1]+.015)<=0 and abs(b[1]-a[1])>1e-9:
            return a+(b-a)*((-.015-a[1])/(b[1]-a[1]))
    return np.full(3,np.nan)

def evaluate(stem,geometry=True):
    d=np.load(ROOT/(stem+'.npz'));poses=d['poses'];h=d['history']
    centers=np.array([[cross_section(q,c) for c in range(COUNT)] for q in poses]);released=h[:,0]>=3.2
    retained=(centers[:,:,0]>.005)&(centers[:,:,0]<.028)&(centers[:,:,2]>(.011+RADIUS-.0002))&(centers[:,:,2]<(.017-RADIUS+.0002))
    worstgap=0.;attachmentgap=0.;wirepenetration=0.;worstpenetration={'top':0.,'bottom':0.}
    meshes={n:trimesh.load(ROOT/('clip'+n.title()+'.stl')) for n in ['top','bottom']}
    for m in meshes.values():m.apply_scale(.001)
    samples=np.linspace(-.0025,.0025,17)
    for pose in poses:
        rods=pose[1:1+COUNT*48];rot=Rotation.from_quat(rods[:,3:]);start=rods[:,:3]+rot.apply(np.tile([0,-.0025,0],(COUNT*48,1)));end=rods[:,:3]+rot.apply(np.tile([0,.0025,0],(COUNT*48,1)))
        worstgap=max(worstgap,float(np.linalg.norm(end.reshape(COUNT,48,3)[:,:-1]-start.reshape(COUNT,48,3)[:,1:],axis=2).max()))
        for cable,dx in enumerate(OFFSETS):
            for endid,endpoint in [(0,start[cable*48]),(1,end[(cable+1)*48-1])]:
                conn=pose[1+COUNT*48+endid]
                # Connector local Z maps to world +/-X in the initial pose.
                local=np.array([0,0,dx*(-1 if endid==0 else 1)])
                anchor=conn[:3]+Rotation.from_quat(conn[3:]).apply(local)
                attachmentgap=max(attachmentgap,float(np.linalg.norm(anchor-endpoint)))
        if geometry:
            direction=rot.apply(np.tile([0,1,0],(COUNT*48,1)))
            pts=(rods[:,:3,None]+direction[:,:,None]*samples).transpose(0,2,1).reshape(-1,3)
            wires=pts.reshape(COUNT,-1,3)
            for c0,c1 in __import__('itertools').combinations(range(COUNT),2):
                distance=cKDTree(wires[c0]).query(wires[c1])[0].min()
                wirepenetration=max(wirepenetration,float(2*RADIUS-distance))
            near=(pts[:,0]>-.055)&(pts[:,0]<.056)&(pts[:,1]>-.035)&(pts[:,1]<.005)&(pts[:,2]<.055)
            pts=pts[near]
            if len(pts):
                for name,m in meshes.items():
                    p=pts if name=='bottom' else Rotation.from_quat(pose[0,3:]).inv().apply(pts-pose[0,:3])
                    signed=trimesh.proximity.signed_distance(m,p)
                    worstpenetration[name]=max(worstpenetration[name],float((RADIUS+signed).max()))
    return {'cable_count':COUNT,'cable_diameter_mm':RADIUS*2000,'clip_scale':CLIP_SCALE,'final_lateral_order_preserved':bool(np.all(np.diff(centers[-1,:,0])>0)),'within_one_diameter_height_band':bool(np.ptp(centers[-1,:,2])<2*RADIUS),'final_height_spread_mm':float(np.ptp(centers[-1,:,2])*1000),'retained_per_cable_during_release':retained[released].all(axis=0).tolist() if released.any() else None,'max_opening_deg':float(-h[:,1].min()),'final_angle_deg':float(h[-1,1]),'max_hinge_anchor_error_mm':float(h[:,2].max()*1000),'max_cable_joint_gap_mm':worstgap*1000,'max_connector_attachment_gap_mm':attachmentgap*1000,'max_sampled_inter_cable_penetration_mm':wirepenetration*1000 if geometry else None,'max_sampled_cable_penetration_mm':({k:v*1000 for k,v in worstpenetration.items()} if geometry else None),'final_cable_cross_section_mm':(centers[-1]*1000).tolist(),'retained_during_release':bool(released.any() and retained[released].all()),'release_observation_seconds':max(0.,float(h[-1,0]-3.2)),'finite':bool(np.isfinite(poses).all())}

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--control',action='store_true');args=parser.parse_args()
    result={'actual_mesh_run':evaluate('actual_motion'),'control_run_included':args.control}
    a=result['actual_mesh_run']
    result['checks']={'finite':a['finite'],'hinge_opens':a['max_opening_deg']>3,'spring_closes':abs(a['final_angle_deg'])<.5,'all_six_cables_retained':a['retained_during_release'],'six_cable_order_preserved':a['final_lateral_order_preserved'],'connectors_attached':a['max_connector_attachment_gap_mm']<.2,'cable_separation':a['max_sampled_inter_cable_penetration_mm']<.2,'hinge_anchors_within_0_05_mm':a['max_hinge_anchor_error_mm']<.05,'cable_joints_within_0_2_mm':a['max_cable_joint_gap_mm']<.2,'sampled_mesh_penetration_below_0_2_mm':max(a['max_sampled_cable_penetration_mm'].values())<.2}
    if args.control:
        b=evaluate('no_gate_contact',False);result['no_gate_contact_control']=b
        result['checks']['contact_disabled_control_stays_closed']=b['finite'] and b['max_opening_deg']<.5
    result['passed']=all(result['checks'].values());(ROOT/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)
    geometry_path=ROOT/'geometry_check.json';geometry=json.loads(geometry_path.read_text())
    geometry['status']='Six-cable geometry, insertion, retention, and attachment checks passed. See validation.json.'
    geometry_path.write_text(json.dumps(geometry,indent=2)+'\n')
