"""Compare identical errors/pushes with feedforward replay and trajectory TVLQR."""
import csv
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
EXE = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else ROOT/"build/svd-check/double_pendulum.exe"
OUT = ROOT/"logs/trajectory_tracking"
CASES = [
    ("nominal", []),
    ("q1_plus_0.1deg", ["--plan-q1-offset-deg", "0.1"]),
    ("q1_minus_0.1deg", ["--plan-q1-offset-deg", "-0.1"]),
    ("q2_plus_0.1deg", ["--plan-q2-offset-deg", "0.1"]),
    ("q2_minus_0.1deg", ["--plan-q2-offset-deg", "-0.1"]),
    ("push_plus_1N", ["--push-force", "1", "--push-start", "1", "--push-duration", "0.1"]),
    ("push_minus_1N", ["--push-force", "-1", "--push-start", "1", "--push-duration", "0.1"]),
    ("late_push_plus_1N", ["--push-force", "1", "--push-start", "2.8", "--push-duration", "0.1"]),
    ("late_push_minus_1N", ["--push-force", "-1", "--push-start", "2.8", "--push-duration", "0.1"]),
    ("offset_and_push", ["--plan-q1-offset-deg", "0.1", "--push-force", "0.5",
                         "--push-start", "1.2", "--push-duration", "0.1"]),
]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    env=os.environ.copy()
    compiler=shutil.which("g++")
    runtime=[str(ROOT/"build/deps/mujoco")]
    if compiler:
        runtime.append(str(Path(compiler).parent))
    elif Path(r"D:\mingw\MinGW\bin").is_dir():
        runtime.append(r"D:\mingw\MinGW\bin")
    env["PATH"]=os.pathsep.join(runtime+[env.get("PATH", "")])
    results=[]
    for name, arguments in CASES:
        for mode in ("plan", "track-plan"):
            output=OUT/f"{name}_{mode}.csv"
            output.unlink(missing_ok=True)
            started=time.perf_counter()
            try:
                run=subprocess.run([str(EXE), f"--{mode}", "config/plans/sqp_swingup.csv", "--headless",
                                    "--duration", "10", "--output", str(output), *arguments],
                                   cwd=ROOT, env=env, capture_output=True, timeout=30)
            except subprocess.TimeoutExpired:
                results.append(dict(case=name, mode=mode, success=False, failure="process_timeout"))
                print(f"{name} {mode}: process timeout", flush=True)
                continue
            elapsed=time.perf_counter()-started
            rows=[]
            if output.exists():
                with output.open(newline="", encoding="utf-8-sig") as stream:
                    rows=[{key:float(value) for key,value in row.items()} for row in csv.DictReader(stream)]
            finite=bool(rows) and all(math.isfinite(v) for row in rows for v in row.values())
            tail=[row for row in rows if row["time"]>=9]
            maxima=[]
            if tail:
                maxima=[max(v) for v in zip(*[(abs(r["p"]), abs(r["dp"]), abs(math.remainder(r["q1"],2*math.pi)),
                    abs(r["dq1"]), abs(math.remainder(r["q1"]+r["q2"],2*math.pi)), abs(r["dq1"]+r["dq2"])) for r in tail])]
            peak_position=max((abs(r["p"]) for r in rows),default=math.inf)
            peak_force=max((abs(r["applied_force"]) for r in rows),default=math.inf)
            peak_cart_speed=max((abs(r["dp"]) for r in rows),default=math.inf)
            peak_joint_speed=max((max(abs(r["dq1"]),abs(r["dq2"])) for r in rows),default=math.inf)
            impulse=sum(r.get("external_force",0)*(r["time"]-previous["time"]) for previous,r in zip(rows,rows[1:]))
            expected_impulse=0.0
            if "--push-force" in arguments:
                expected_impulse=float(arguments[arguments.index("--push-force")+1])*float(arguments[arguments.index("--push-duration")+1])
            push_valid=abs(impulse-expected_impulse)<1e-8
            success=(run.returncode==0 and finite and abs(rows[-1]["time"]-10)<0.003 and len(tail)>=499 and
                     len(maxima)==6 and all(value<limit for value,limit in zip(maxima,(.05,.05,.02,.05,.02,.05))) and
                     peak_position<3.85 and peak_force<=20+1e-9 and peak_cart_speed<=8 and peak_joint_speed<=30 and push_valid and
                     b"Offline SQP plan -> LQR at t=4" in run.stdout)
            setup=re.search(rb"setup ([\d.eE+-]+) s",run.stdout)
            result=dict(case=name, mode=mode, success=success, returncode=run.returncode,
                        wall_seconds=elapsed, setup_seconds=float(setup[1]) if setup else None,
                        final_time=rows[-1]["time"] if rows else None, peak_position_m=peak_position,
                        peak_force_N=peak_force, tail_max_errors=maxima, external_impulse_Ns=impulse,
                        expected_impulse_Ns=expected_impulse, push_valid=push_valid)
            if not success:
                result["stderr"]=run.stderr.decode("utf-8",errors="replace")
            results.append(result)
            print(f"{name} {mode}: success={success} peak_p={peak_position:.4f} impulse={impulse:.4f}",flush=True)
    report=dict(scope="Fixed SQP reference + TVLQR; finite scenarios, not online NMPC or a global guarantee",cases=results)
    (OUT/"report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    tracked=[r for r in results if r["mode"]=="track-plan"]
    print(f"Tracking passed {sum(r['success'] for r in tracked)}/{len(tracked)} scenarios")
    return 0 if len(tracked)==len(CASES) and all(r["success"] for r in tracked) else 1


if __name__=="__main__":
    sys.exit(main())
