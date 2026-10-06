"""Run every audit, preserving all reports even when one check fails."""
from pathlib import Path
import subprocess,sys,json
from concurrent.futures import ThreadPoolExecutor
root=Path(__file__).resolve().parent
checks=['validate.py','check_clip_contacts.py','check_gripper_contacts.py','check_connector_contacts.py','check_robot_clearance.py','check_recovery_policy.py'];codes={}
def run_check(script):
 with (root/(script.removesuffix('.py')+'.log')).open('w') as log:
  return subprocess.run([sys.executable,str(root/script)],cwd=root,stdout=log,stderr=subprocess.STDOUT).returncode
with ThreadPoolExecutor(max_workers=4) as pool:codes=dict(zip(checks,pool.map(run_check,checks)))
print(json.dumps(codes,indent=2),flush=True)
(root/'validation_summary.json').write_text(json.dumps({'passed':not any(codes.values()),'exit_codes':codes},indent=2)+'\n')
raise SystemExit(1 if any(codes.values()) else 0)
