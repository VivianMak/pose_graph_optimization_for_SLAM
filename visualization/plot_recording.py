import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

DIR = "data/drive_in_square_points/"

# Odometry Data
# from bag recording
odom_data = pd.read_csv(DIR + "odom_data.csv")
odom_xs = odom_data["x"].values
odom_ys = odom_data["y"].values

# added noise
noisy_data = pd.read_csv(DIR + "noisy_odom_data.csv")
noisy_xs = noisy_data["x"].values
noisy_ys = noisy_data["y"].values

limit = 500


# Lidar Scans
data = np.loadtxt(DIR + "lidar_scans.csv", delimiter=",")


scan = data[0]   # first scan
scan = np.where(np.isfinite(scan), scan, 0.0)
angles = np.linspace(0, 2*np.pi, len(scan), endpoint=False)
scan_x = scan * np.cos(angles)
scan_y = scan * np.sin(angles)

# scan2 = data[2695]   # second scan
# scan2 = np.where(np.isfinite(scan2), scan2, 0.0)
# angles2 = np.linspace(0, 2*np.pi, len(scan2), endpoint=False)
# scan_x2 = scan2 * np.cos(angles)
# scan_y2 = scan2 * np.sin(angles)

plt.figure()

plt.scatter(scan_x, scan_y, s=5, c='blue')
# plt.scatter(scan_x2, scan_y2, s=5, c='red')

# plt.plot(odom_xs[:limit], odom_ys[:limit])
# plt.plot(noisy_xs[:limit], noisy_ys[:limit])
plt.plot(odom_xs, odom_ys)
# plt.plot(noisy_xs, noisy_ys)

plt.axis([-7, 7, -7, 7])
# plt.axis("equal")

plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Lidar Scans")

plt.show()
