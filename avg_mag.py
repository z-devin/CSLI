import numpy as np

data = np.loadtxt('mag_data.csv', delimiter=',', skiprows=1)
avg = np.mean(data, axis=0)
print(avg)
#-0.70553762  0.10511136  0.70082871]