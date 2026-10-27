"""Plot actual SQP timings and distinguish optimization from fallback."""
import argparse
import csv
import os
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"build/deps/python_plot"))
os.environ.setdefault("MPLCONFIGDIR",str(ROOT/"build/matplotlib_cache"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("csv",type=Path)
parser.add_argument("--output",type=Path,default=Path("results/solver_timing.png"))
args=parser.parse_args()
with args.csv.open(newline="",encoding="utf-8-sig") as stream:
    rows=list(csv.DictReader(stream))
if not rows:
    raise ValueError("Empty SQP telemetry CSV")
t=[float(r["time"]) for r in rows]
seconds=[float(r["solve_seconds"])*1000 for r in rows]
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
axes[0].plot(t,seconds,".-",label="Measured solve time")
axes[0].plot(t,[float(r["control_period"])*1000 for r in rows],"--",label="Control period")
axes[0].plot(t,[float(r["budget_seconds"])*1000 for r in rows],":",label="Soft solve budget")
axes[0].set_ylabel("Wall time (ms)")
axes[1].plot(t,[float(r["initial_cost"])-float(r["final_cost"]) for r in rows],".-",label="Cost reduction per solve")
axes[1].set_ylabel("Cost reduction")
codes={"trajectory_feedback":0,"feasible_warm_start":1,"sqp":2}
axes[2].step(t,[codes[r["selected"]] for r in rows],where="post")
axes[2].set_yticks([0,1,2],["TVLQR fallback","Feasible warm start","SQP update"])
axes[2].set_xlabel("Simulation time (s)")
for ax in axes:
    ax.grid(alpha=.25)
for ax in axes[:2]:
    ax.legend()
fig.suptitle("Seeded online SQP: measured timings and selected control")
args.output.parent.mkdir(parents=True,exist_ok=True)
fig.savefig(args.output,dpi=150)
plt.close(fig)
print(args.output.resolve())
