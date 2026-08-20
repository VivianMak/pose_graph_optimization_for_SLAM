import os
import pandas as pd
import matplotlib.pyplot as plt

# Resolve paths relative to this script, not the caller's cwd, so this
# works whether you run it from repo root or from visualization/.
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")

PRE_COLOR = "#C44E52"    # muted crimson
OPT_COLOR = "#4C72B0"    # muted blue
NOISY_COLOR = "#999999"  # neutral gray, kept low-contrast since it's background data

pre = pd.read_csv(os.path.join(DATA_DIR, "pre_optimized.csv"))
opt = pd.read_csv(os.path.join(DATA_DIR, "optimized.csv"))
noisy_odom = pd.read_csv(os.path.join(DATA_DIR, "drive_in_square_points", "noisy_odom_data.csv"))

fig, ax = plt.subplots(figsize=(8, 8))

ax.scatter(
    noisy_odom["x"].values, noisy_odom["y"].values,
    s=5, c=NOISY_COLOR, alpha=0.5, linewidths=0,
    label="recorded odom", zorder=1,
)

ax.plot(
    pre["x"].values, pre["y"].values,
    color=PRE_COLOR, linewidth=2, label="pre-optimized (noisy)", zorder=2,
)
ax.plot(
    opt["x"].values, opt["y"].values,
    color=OPT_COLOR, linewidth=2, label="optimized", zorder=3,
)

# Start markers so the direction of travel is readable at a glance
ax.scatter(
    pre["x"].values[0], pre["y"].values[0],
    s=70, facecolor="white", edgecolor=PRE_COLOR, linewidth=2, zorder=4,
)
ax.scatter(
    opt["x"].values[0], opt["y"].values[0],
    s=70, facecolor="white", edgecolor=OPT_COLOR, linewidth=2, zorder=4,
)

ax.set_aspect("equal")

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Pose Graph: Pre- vs Post-Optimization", fontsize=14, fontweight="bold")

ax.grid(True, linestyle="--", alpha=0.3)
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)

ax.legend(frameon=False, loc="best")

fig.tight_layout()
fig.savefig(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "optimizer.png"),
    dpi=150,
)
plt.show()
