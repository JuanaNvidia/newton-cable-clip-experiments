"""Validate the qualitative arch/recovery target from the recorded Newton test."""
from pathlib import Path
import hashlib,json
import numpy as np
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'arch_results.json').read_text());p=np.load(ROOT/'arch_motion.npz')['poses'];old=r['results'][1];selected=r['results'][2]
checks={
 'finite_recorded_poses':bool(np.isfinite(p).all()),
 'selected_arch_above_endpoints_by_50_mm':selected['final_apex_mm']-r['endpoint_elevation_m']*1000>50,
 'selected_arch_survives_load':selected['min_apex_during_press_mm']-r['endpoint_elevation_m']*1000>50,
 'recovers_within_0_1_mm':abs(selected['final_apex_mm']-selected['apex_at_3s_mm'])<.1,
 'settled_within_0_1_mm_in_last_second':selected['last_second_apex_range_mm']<.1,
 'firmer_than_previous_material':selected['apex_at_3s_mm']-selected['min_apex_during_press_mm']<old['apex_at_3s_mm']-old['min_apex_during_press_mm'],
 'anchors_within_0_05_mm':max(x['max_end_anchor_error_mm'] for x in r['results'])<.05,
 'joint_gaps_within_0_05_mm':max(x['max_joint_gap_mm'] for x in r['results'])<.05,
 'mass_matches_solid_cylinder':abs(selected['mass_g']-1000*np.pi*.001**2*.3048*1000)<1e-5,
}
report={'checks':checks,'passed':all(checks.values()),'motion_sha256':hashlib.sha256((ROOT/'arch_motion.npz').read_bytes()).hexdigest(),'limitation':'Qualitative target and assumed connector boundary conditions; not measured material identification.'}
(ROOT/'arch_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));assert report['passed']
