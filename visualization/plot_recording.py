import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Resolve paths relative to this script, not the caller's cwd, so this
# works whether you run it from repo root or from visualization/.
DATA_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "data", "drive_in_square_points"
)

SCAN_COLOR = "#4C72B0"      # muted blue
ODOM_COLOR = "#C44E52"      # muted crimson
NOISY_COLOR = "#8172B2"     # muted purple, unused by default -- see below

# --- Odometry (from bag recording) ---
odom_data = pd.read_csv(os.path.join(DATA_DIR, "odom_data.csv"))
odom_xs = odom_data["x"].values
odom_ys = odom_data["y"].values

# --- Added noise (uncomment the plot call below to overlay it) ---
noisy_data = pd.read_csv(os.path.join(DATA_DIR, "noisy_odom_data.csv"))
noisy_xs = noisy_data["x"].values
noisy_ys = noisy_data["y"].values

# --- LiDAR scan ---
scans = np.loadtxt(os.path.join(DATA_DIR, "lidar_scans.csv"), delimiter=",")

scan = scans[0]  # first scan
scan = np.where(np.isfinite(scan), scan, 0.0)
angles = np.linspace(0, 2 * np.pi, len(scan), endpoint=False)
scan_x = scan * np.cos(angles)
scan_y = scan * np.sin(angles)

# A second scan, for comparing two vantage points -- disabled by default.
# scan2 = scans[2695]
# scan2 = np.where(np.isfinite(scan2), scan2, 0.0)
# angles2 = np.linspace(0, 2 * np.pi, len(scan2), endpoint=False)
# scan_x2 = scan2 * np.cos(angles2)
# scan_y2 = scan2 * np.sin(angles2)

fig, ax = plt.subplots(figsize=(8, 8))

ax.scatter(
    scan_x, scan_y,
    s=6, c=SCAN_COLOR, alpha=0.6, linewidths=0,
    label="LiDAR scan (t=0)", zorder=2,
)
# ax.scatter(scan_x2, scan_y2, s=6, c="#DD8452", alpha=0.6, linewidths=0,
#            label="LiDAR scan (t=2695)", zorder=2)

ax.plot(
    odom_xs, odom_ys,
    color=ODOM_COLOR, linewidth=2, label="Odometry path", zorder=3,
)
# ax.plot(noisy_xs, noisy_ys, color=NOISY_COLOR, linewidth=2,
#         linestyle="--", label="Noisy odometry", zorder=3)

# Start marker so the direction of travel is readable at a glance
ax.scatter(
    odom_xs[0], odom_ys[0],
    s=70, facecolor="white", edgecolor=ODOM_COLOR, linewidth=2,
    zorder=4, label="Start",
)

ax.set_xlim(-7, 7)
ax.set_ylim(-7, 7)
ax.set_aspect("equal")

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Recorded Drive: Odometry + LiDAR Scan", fontsize=14, fontweight="bold")

ax.grid(True, linestyle="--", alpha=0.3)
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)

ax.legend(frameon=False, loc="upper right")

fig.tight_layout()
fig.savefig(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "recording.png"),
    dpi=150,
)
plt.show()
