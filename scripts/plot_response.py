"""Plot physical coordinates and absolute pole orientations from simulation CSV."""
import argparse
import csv
import math
import os
from pathlib import Path
import sys
project_root = Path(__file__).resolve().parents[1]
local_packages = project_root / "build/deps/python_plot"
if local_packages.is_dir():
    sys.path.insert(0, str(local_packages))
os.environ.setdefault("MPLCONFIGDIR", str(project_root / "build/matplotlib_cache"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("csv", type=Path)
parser.add_argument("--output", type=Path, default=Path("results/swingup_response.png"))
parser.add_argument("--switch-time", type=float, default=4.0)
parser.add_argument("--compare", type=Path, help="Optional second response, shown dashed (may stop early)")
parser.add_argument("--title", default="Offline SQP swing-up followed by LQR (nominal model)")
args = parser.parse_args()
with args.csv.open(newline="", encoding="utf-8-sig") as stream:
    rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
if not rows:
    raise ValueError("Empty response CSV")
t = [r["time"] for r in rows]
has_stage="stage" in rows[0]
fig, axes = plt.subplots(5 if has_stage else 4, 1, figsize=(10, 12 if has_stage else 10), sharex=True, constrained_layout=True)
axes[0].plot(t, [r["p"] for r in rows], label="Cart position")
axes[0].axhline(3.85, color="r", linestyle=":")
axes[0].axhline(-3.85, color="r", linestyle=":", label="Planning boundary")
axes[0].set_ylabel("Position (m)")
axes[1].plot(t, [math.degrees(r["q1"]) for r in rows], label="Pole 1")
axes[1].plot(t, [math.degrees(r["q1"]+r["q2"]) for r in rows], label="Pole 2 absolute")
axes[1].set_ylabel("Angle (deg, continuous)")
axes[2].plot(t, [r["dq1"] for r in rows], label="Pole 1 speed")
axes[2].plot(t, [r["dq1"]+r["dq2"] for r in rows], label="Pole 2 absolute speed")
axes[2].set_ylabel("Speed (rad/s)")
axes[3].plot(t, [r["applied_force"] for r in rows], label="Applied force")
if any(r.get("external_force",0)!=0 for r in rows):
    axes[3].plot(t, [r.get("external_force",0) for r in rows], label="External push", color="purple")
if args.compare:
    with args.compare.open(newline="", encoding="utf-8-sig") as stream:
        other=[{key:float(value) for key,value in row.items()} for row in csv.DictReader(stream)]
    ot=[r["time"] for r in other]
    axes[0].plot(ot,[r["p"] for r in other],"--",alpha=.7,label="Feedforward comparison")
    axes[1].plot(ot,[math.degrees(r["q1"]) for r in other],"--",alpha=.7,label="Feedforward pole 1")
    axes[1].plot(ot,[math.degrees(r["q1"]+r["q2"]) for r in other],"--",alpha=.7,label="Feedforward pole 2")
    axes[2].plot(ot,[r["dq1"] for r in other],"--",alpha=.7,label="Feedforward pole 1")
    axes[2].plot(ot,[r["dq1"]+r["dq2"] for r in other],"--",alpha=.7,label="Feedforward pole 2")
    axes[3].plot(ot,[r["applied_force"] for r in other],"--",alpha=.7,label="Feedforward input")
axes[3].axhline(20, color="r", linestyle=":")
axes[3].axhline(-20, color="r", linestyle=":")
axes[3].set_ylabel("Force (N)")
axes[3].set_xlabel("Simulation time (s)")
if has_stage:
    axes[4].step(t,[r["stage"] for r in rows],where="post",label="Control stage")
    axes[4].set_yticks([0,1,2,3,4,5,6,7],["Open loop","Plan","SQP","LQR","Fallback","TVLQR","LMPC","Energy test"])
    axes[4].set_xlabel("Simulation time (s)")
for ax in axes:
    ax.axvline(args.switch_time, color="k", linestyle="--", alpha=.5, label="LQR capture" if ax is axes[0] else None)
    ax.grid(alpha=.25)
    ax.legend(loc="best")
fig.suptitle(args.title)
args.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.output, dpi=150)
plt.close(fig)
print(args.output.resolve())
