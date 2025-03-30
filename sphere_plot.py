from matplotlib import pyplot as plt
import numpy as np

data = np.loadtxt("data2.csv", delimiter=",")

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

def plot_sphere(ax, center, radius):
    u = np.linspace(0, 2*np.pi, 10)
    v = np.linspace(0, np.pi, 10)

    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius*np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius*np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(x, y, z, color="b", alpha=0.6)


# old values:
# d = np.array([x_data, y_data, z_data])
A = np.array([[0.9318, -0.0353, -0.0041],
                [-0.0353, 0.9532, 0.0912],
                [-0.0041, 0.0912, 1.1362]])
b = np.array([3.274e3, -7.9187e3, -1.6536e4])

# new values:
A = np.array([[0.9413, -0.0043, 0.0376],
                [-0.0043, 0.9909, 0.0086],
                [0.0376, 0.0086, 1.0737]])

b = np.array([3.3744e3, -7.7696e3, -1.6720e4])

out=[(d-b)@A for d in data]

# data = np.array(out)

ax.scatter(data[:,0], data[:,1], data[:,2], marker="o")

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

# plot_sphere(ax, [0, 0, 0], 4.5271e3)
ax.set_aspect("equal")

plt.show()