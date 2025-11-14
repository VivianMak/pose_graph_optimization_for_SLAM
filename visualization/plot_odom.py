import pandas as pd
import matplotlib.pyplot as plt
odom_data = pd.read_csv("odom_data.csv")
odom_xs = odom_data["x"].values
odom_ys = odom_data["y"].values

noisy_data = pd.read_csv("noisy_odom_data.csv")
noisy_xs = noisy_data["x"].values
noisy_ys = noisy_data["y"].values

plt.plot(odom_xs, odom_ys)
plt.plot(noisy_xs, noisy_ys)
plt.axis("equal")
plt.show()
