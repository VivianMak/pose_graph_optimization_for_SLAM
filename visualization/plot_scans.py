import numpy as np
import matplotlib.pyplot as plt

# Load CSV
data = np.loadtxt("lidar_scans.csv", delimiter=",")

# pick a scan to visualize
scan = data[100]   # first scan
scan = np.where(np.isfinite(scan), scan, 0.0)
angles = np.linspace(0, 2*np.pi, len(scan), endpoint=False)
xs = scan * np.cos(angles)
ys = scan * np.sin(angles)

scan2 = data[3150]   # second scan
scan2 = np.where(np.isfinite(scan2), scan2, 0.0)
angles2 = np.linspace(0, 2*np.pi, len(scan2), endpoint=False)
xs2 = scan2 * np.cos(angles2)
ys2 = scan2 * np.sin(angles2)

plt.figure()
plt.scatter(xs, ys, s=5, c='blue')  # s=point size
plt.scatter(xs2, ys2, s=5, c='red') 
plt.axis("equal")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Lidar Scans")
plt.show()