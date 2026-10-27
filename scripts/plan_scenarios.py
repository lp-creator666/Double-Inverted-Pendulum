"""Regenerate the scenario library into logs; promote verified results manually."""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"tests/acceptance"))
from check_project_acceptance import runtime_env

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--exe",type=Path,default=ROOT/"build/svd-check/plan_swingup.exe")
parser.add_argument("--iterations",type=int,default=20)
args=parser.parse_args()
if not 1<=args.iterations<=1000:
    parser.error("iterations must be in [1,1000]")
directory=ROOT/"logs/planning"
directory.mkdir(parents=True,exist_ok=True)
scenarios=json.loads((ROOT/"config/scenarios.json").read_text(encoding="utf-8"))["scenarios"]
results=[]
for scenario in scenarios:
    if scenario["name"]=="down_negative":
        # This entry is the exact sign symmetry of the nominal model, not a new solve.
        continue
    name=scenario["name"]
    seed=ROOT/("config/seeds/sqp_swingup_seed.csv" if name=="down" else f"config/seeds/{name}_seed.csv")
    run=subprocess.run([str(args.exe.resolve()),str(math.radians(scenario["q1_deg"])),str(seed),str(args.iterations),
                        str(math.radians(scenario["q2_deg"]))],cwd=ROOT,env=runtime_env(),capture_output=True,timeout=600)
    text=run.stdout.decode("utf-8",errors="replace")
    (directory/f"{name}.txt").write_text(text+run.stderr.decode("utf-8",errors="replace"),encoding="utf-8")
    verified=ROOT/"logs/swingup_verified.csv"
    output=directory/f"{name}.csv"
    output.unlink(missing_ok=True)
    passed=run.returncode==0 and verified.exists() and "continuous_replay_and_lqr_stable=1" in text
    if passed:
        shutil.copyfile(verified,output)
    result=dict(case=name,success=passed,returncode=run.returncode,plan=str(output) if passed else None)
    results.append(result);print(json.dumps(result),flush=True)
(directory/"report.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
sys.exit(0 if all(r["success"] for r in results) else 1)
