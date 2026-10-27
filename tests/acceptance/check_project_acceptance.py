"""End-to-end controls, disturbances, solver telemetry, and deliberate failures."""
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
EXE = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else ROOT/"build/svd-check/double_pendulum.exe"
OUT = ROOT/"logs/acceptance"


def runtime_env():
    env=os.environ.copy()
    compiler=shutil.which("g++")
    paths=[str(ROOT/"build/deps/mujoco")]
    if compiler:
        paths.append(str(Path(compiler).parent))
    elif Path(r"D:\mingw\MinGW\bin").is_dir():
        paths.append(r"D:\mingw\MinGW\bin")
    env["PATH"]=os.pathsep.join(paths+[env.get("PATH", "")])
    return env


def read_rows(path):
    if not path.exists():
        return []
    with path.open(newline="",encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def run_case(name,args,expected_error=None,require_optimization=False,require_timeout=False):
    output=OUT/f"{name}.csv"
    diagnostics=Path(str(output)+".solver.csv")
    output.unlink(missing_ok=True)
    diagnostics.unlink(missing_ok=True)
    started=time.perf_counter()
    try:
        run=subprocess.run([str(EXE),"--headless","--duration","10","--output",str(output),*args],
                           cwd=ROOT,env=runtime_env(),capture_output=True,timeout=90)
    except subprocess.TimeoutExpired:
        return dict(case=name,success=False,reason="process_timeout")
    result=dict(case=name,returncode=run.returncode,wall_seconds=time.perf_counter()-started)
    stderr=run.stderr.decode("utf-8",errors="replace")
    if expected_error:
        result.update(success=run.returncode!=0 and expected_error in stderr,expected_failure=expected_error,stderr=stderr)
        return result
    rows=[{k:float(v) for k,v in row.items()} for row in read_rows(output)]
    finite=bool(rows) and all(math.isfinite(v) for r in rows for v in r.values())
    tail=[r for r in rows if r["time"]>=9]
    maxima=[]
    if tail:
        maxima=[max(v) for v in zip(*[(abs(r["p"]),abs(r["dp"]),abs(math.remainder(r["q1"],2*math.pi)),
            abs(r["dq1"]),abs(math.remainder(r["q1"]+r["q2"],2*math.pi)),abs(r["dq1"]+r["dq2"])) for r in tail])]
    peak_p=max((abs(r["p"]) for r in rows),default=math.inf)
    peak_u=max((abs(r["applied_force"]) for r in rows),default=math.inf)
    peak_dp=max((abs(r["dp"]) for r in rows),default=math.inf)
    peak_dq=max((max(abs(r["dq1"]),abs(r["dq2"])) for r in rows),default=math.inf)
    impulse=sum(r["external_force"]*(r["time"]-p["time"]) for p,r in zip(rows,rows[1:]))
    expected_impulse=0
    if "--push-force" in args:
        expected_impulse=float(args[args.index("--push-force")+1])*float(args[args.index("--push-duration")+1])
    ok=(run.returncode==0 and finite and abs(rows[-1]["time"]-10)<.003 and len(tail)>=499 and
        len(maxima)==6 and all(v<limit for v,limit in zip(maxima,(.05,.05,.02,.05,.02,.05))) and
        peak_p<3.85 and peak_u<=20+1e-9 and peak_dp<=8 and peak_dq<=30 and
        abs(impulse-expected_impulse)<1e-8 and rows[-1].get("stage")== (6 if "--lmpc" in args else 3))
    solver=read_rows(diagnostics)
    if "--online-plan" in args:
        selected=[r for r in solver if r["selected"]=="sqp"]
        reused=[r for r in solver if r["selected"]=="feasible_warm_start"]
        seconds=sorted(float(r["solve_seconds"]) for r in solver)
        deadline_misses=sum(int(r["deadline_missed"]) for r in solver)
        timeout_count=sum(r["status"]=="timeout" for r in solver)
        log_ok=(len(solver)==50 and all(int(r["horizon"])==50-i for i,r in enumerate(solver)) and
            all(float(r["constraint_violation"])<=1e-5 and r["feasible"]=="1" and
                float(r["solve_seconds"])<=float(r["budget_seconds"])
                and (int(r["accepted_steps"])>0 or r["status"]=="converged") for r in selected))
        log_ok=log_ok and all(float(r["constraint_violation"])<=1e-5 and r["feasible"]=="1" and
            r["status"]!="timeout" and r["budget_exceeded"]=="0" for r in reused)
        if require_optimization:
            log_ok=log_ok and len(selected)>0
        if require_timeout:
            log_ok=log_ok and timeout_count==50 and not selected and all(r["selected"]=="trajectory_feedback" for r in solver)
        ok=ok and log_ok
        result["solver"]=dict(calls=len(solver),selected=len(selected),warm_start_reuses=len(reused),fallbacks=len(solver)-len(selected)-len(reused),
            timeouts=timeout_count,deadline_misses=deadline_misses,
            mean_seconds=sum(seconds)/len(seconds) if seconds else None,
            p95_seconds=seconds[min(len(seconds)-1,math.ceil(.95*len(seconds))-1)] if seconds else None,
            max_seconds=max(seconds) if seconds else None,valid=log_ok)
    result.update(success=ok,peak_position_m=peak_p,peak_force_N=peak_u,peak_cart_speed=peak_dp,
                  peak_joint_speed=peak_dq,tail_max_errors=maxima,external_impulse_Ns=impulse)
    if not ok:
        result["stderr"]=stderr
    return result


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    plan="config/plans/sqp_swingup.csv"
    cases=[]
    cases.append(("lqr_local",["--lqr","--q1",str(math.radians(1))]))
    cases.append(("lmpc_local",["--lmpc","--q1",str(math.radians(1))]))
    for i,axis in enumerate(("q1","q2")):
        for sign in (-1,1):
            cases.append((f"track_{axis}_{sign*2}deg",["--track-plan",plan,f"--plan-{axis}-offset-deg",str(sign*2)]))
    for axis in ("dp","dq1","dq2"):
        for sign in (-1,1):
            cases.append((f"track_{axis}_{sign*0.1}",["--track-plan",plan,f"--plan-{axis}-offset",str(sign*.1)]))
    for sign in (-1,1):
        cases.append((f"track_push_{sign*2}N",["--track-plan",plan,"--push-force",str(sign*2),"--push-start","1","--push-duration","0.1"]))
    for name in ("down_negative","down_q2_20","down_q2_90"):
        cases.append((f"track_{name}",["--track-plan",f"config/scenarios/{name}.csv"]))
    for name,args in (("nominal",[]),("q1_error",["--plan-q1-offset-deg","0.1"]),
                      ("q2_error",["--plan-q2-offset-deg","0.1"]),
                      ("velocity",["--plan-dq1-offset","0.02"]),
                      ("push",["--push-force","1","--push-start","1","--push-duration","0.1"]),
                      ("late_push",["--push-force","-1","--push-start","2.8","--push-duration","0.1"])):
        cases.append((f"online_{name}",["--online-plan",plan,*args]))
    cases.append(("online_down_q2_90",["--online-plan","config/scenarios/down_q2_90.csv"]))
    results=[]
    for name,args in cases:
        r=run_case(name,args,require_optimization=name.startswith("online_"))
        results.append(r);print(json.dumps(r,ensure_ascii=False),flush=True)
    r=run_case("timeout_fallback",["--online-plan",plan,"--solve-budget","0.000000001"],require_timeout=True)
    results.append(r);print(json.dumps(r,ensure_ascii=False),flush=True)
    bad_model=OUT/"inconsistent_plan.csv"
    records=read_rows(ROOT/plan)
    records[1]["p"]=str(float(records[1]["p"])+0.01)
    with bad_model.open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=records[0].keys());writer.writeheader();writer.writerows(records)
    failures=[("large_departure",["--track-plan",plan,"--plan-q1-offset-deg","90"],"validity guard"),
              ("model_mismatch",["--track-plan",str(bad_model)],"inconsistent with current model"),
              ("conflicting_modes",["--lqr","--lmpc"],"mutually exclusive"),
              ("missing_plan",["--online-plan",str(OUT/"does_not_exist.csv")],"Cannot read swing-up plan"),
              ("bad_budget",["--online-plan",plan,"--solve-budget","-1"],"Positive --solve-budget")]
    for name,args,message in failures:
        r=run_case(name,args,expected_error=message);results.append(r);print(json.dumps(r,ensure_ascii=False),flush=True)
    report=dict(scope="Finite tested scenarios in current XML; online SQP is seeded, shrinking-horizon, soft real-time",cases=results,
                passed=sum(r["success"] for r in results),total=len(results))
    (OUT/"report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(f"Acceptance: {report['passed']}/{report['total']}")
    return 0 if report["passed"]==report["total"] else 1


if __name__=="__main__":
    sys.exit(main())
