"""Copy an experiment to a fresh directory and run selected stages there."""
from pathlib import Path
import argparse,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
CASES={
 'ur5-interior-clip':('ur5_interior_clip',{'simulate':[['simulate.py']],'validate':[['validate.py'],['check_clip_contacts.py'],['check_gripper_contacts.py'],['check_robot_clearance.py']],'render':[['render.py']],'usd':[['export_playback.py']]}),
 'ur5-clip':('ur5_cable_clip',{'simulate':[['simulate.py']],'validate':[['validate.py'],['check_clip_contacts.py'],['check_robot_clearance.py']],'render':[['render.py']],'usd':[['export_playback.py']]}),
 'connector-and-clip':('connector_and_clip',{'simulate':[['assets.py'],['simulate.py']],'validate':[['validate.py']],'render':[['render.py']],'usd':[['export_playback.py']]}),
 'fixed-connector':('fixed_receiver_insertion',{'simulate':[['assets.py'],['simulate.py']],'validate':[['validate.py']],'render':[['render.py']],'usd':[['export_playback.py']]}),
 'hinged-connector':('hinged_connector_insertion',{'simulate':[['assets.py'],['simulate.py'],['simulate.py','--no-receiver-contact','--output','no_receiver_contact']],'validate':[['validate.py','--control']],'render':[['render.py']],'usd':[['export_playback.py']]}),
 'insertion':('six_cable_newton',{'simulate':[['simulate_newton.py'],['simulate_newton.py','--seconds','3','--no-gate-contact','--output','no_gate_contact']],'validate':[['validate_actual.py','--control']],'render':[['render_newton.py'],['make_video.py']],'usd':[['export_playback.py']]}),
 'long-insertion':('long_cable_newton',{'simulate':[['simulate_newton.py']],'validate':[['validate_actual.py']],'render':[['render_newton.py']],'usd':[['export_playback.py']]}),
 'arch':('long_cable_newton',{'simulate':[['arch_test.py']],'validate':[['validate_arch.py']],'render':[['render_arch.py']]}),
 'three-inch':('three_inch_arch',{'simulate':[['arch_test.py']],'validate':[['validate_motion.py']],'render':[['render_arch.py']]})}
p=argparse.ArgumentParser(description=__doc__);p.add_argument('experiment',choices=CASES);p.add_argument('--output',type=Path,required=True);p.add_argument('--steps',nargs='+',choices=['simulate','validate','render','usd'],default=['simulate','validate','render']);a=p.parse_args()
folder,steps=CASES[a.experiment]
for step in a.steps:
 if step not in steps:p.error(f'{step} is not supported for {a.experiment}')
out=a.output.resolve()
if out.exists():p.error('Output directory already exists; choose a fresh path to preserve previous results.')
shutil.copytree(ROOT/'experiments'/folder,out)
for step in a.steps:
 for command in steps[step]:
  print('+',sys.executable,*command,flush=True);subprocess.run([sys.executable,*command],cwd=out,check=True)
print('Results:',out)
