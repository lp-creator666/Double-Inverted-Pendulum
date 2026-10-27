"""Validate nominal offline SQP replay and LQR capture; not online NMPC."""
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
EXE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "build/svd-check/double_pendulum.exe"
OUT = ROOT / "logs/planned_swingup"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    compiler = shutil.which("g++")
    runtime = [str(ROOT / "build/deps/mujoco")]
    if compiler:
        runtime.append(str(Path(compiler).parent))
    elif Path(r"D:\mingw\MinGW\bin").is_dir():
        runtime.append(r"D:\mingw\MinGW\bin")
    env["PATH"] = os.pathsep.join(runtime + [env.get("PATH", "")])
    with (ROOT / "config/plans/sqp_swingup.csv").open(newline="", encoding="utf-8-sig") as stream:
        source = list(csv.DictReader(stream))
    results = []
    for name, sign in (("down_positive", 1), ("down_negative", -1)):
        plan = OUT / f"{name}_plan.csv"
        with plan.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=source[0].keys())
            writer.writeheader()
            writer.writerows({key: float(value) * (1 if key == "time" else sign)
                              for key, value in row.items()} for row in source)
        output = OUT / f"{name}.csv"
        # Prevent a previous successful run from masquerading as this result.
        output.unlink(missing_ok=True)
        run = subprocess.run([str(EXE), "--plan", str(plan), "--headless", "--duration", "10",
                              "--output", str(output)], cwd=ROOT, env=env,
                             capture_output=True, timeout=30)
        rows = []
        if output.exists():
            with output.open(newline="", encoding="utf-8-sig") as stream:
                rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
        finite = bool(rows) and all(math.isfinite(v) for row in rows for v in row.values())
        tail = [row for row in rows if row["time"] >= 9]
        def errors(row):
            return (abs(row["p"]), abs(row["dp"]), abs(math.remainder(row["q1"], 2*math.pi)),
                    abs(row["dq1"]), abs(math.remainder(row["q1"]+row["q2"], 2*math.pi)),
                    abs(row["dq1"]+row["dq2"]))
        maxima = [max(values) for values in zip(*(errors(row) for row in tail))] if tail else []
        peak_position = max((abs(row["p"]) for row in rows), default=math.inf)
        peak_force = max((abs(row["applied_force"]) for row in rows), default=math.inf)
        peak_cart_speed = max((abs(row["dp"]) for row in rows), default=math.inf)
        peak_joint_speed = max((max(abs(row["dq1"]), abs(row["dq2"])) for row in rows), default=math.inf)
        success = (run.returncode == 0 and finite and abs(rows[-1]["time"]-10)<0.003 and
                   len(tail)>=499 and len(maxima)==6 and
                   all(value < limit for value, limit in zip(maxima, (.05,.05,.02,.05,.02,.05))) and
                   peak_position < 3.85 and peak_force <= 20+1e-9 and
                   peak_cart_speed <= 8 and peak_joint_speed <= 30 and
                   b"Offline SQP plan -> LQR at t=4" in run.stdout)
        result = dict(case=name, success=success, returncode=run.returncode,
                      peak_position_m=peak_position, peak_force_N=peak_force,
                      peak_cart_speed=peak_cart_speed, peak_joint_speed=peak_joint_speed,
                      tail_max_errors=maxima)
        if run.returncode:
            result["stderr"] = run.stderr.decode("utf-8", errors="replace")
        results.append(result)
        print(json.dumps(result, ensure_ascii=False))
    (OUT / "report.json").write_text(json.dumps({"scope": "nominal feedforward replay + LQR; symmetric model",
        "cases": results}, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if all(result["success"] for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
