import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation
import numpy as np

class CubeVisualizer:
    def __init__(self, context):
        self.context = context
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.cube_faces = self.create_cube()
        self.animation = None

    def create_cube(self):
        vertices = np.array([
            [1, -1, -1],
            [1, 1, -1],
            [-1, 1, -1],
            [-1, -1, -1],
            [1, -1, 1],
            [1, 1, 1],
            [-1, 1, 1],
            [-1, -1, 1]
        ])
        faces = np.array([
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # front
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # back
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # right
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # left
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # top
            [vertices[0], vertices[3], vertices[7], vertices[4]]   # bottom
        ])
        return faces

    def update(self, frame):
        self.ax.cla()
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        q_current = self.context.q_current
        R = Rotation.from_quat(q_current).as_matrix()

        rotated_faces = np.array([[R @ vertex for vertex in face] for face in self.cube_faces])
        for face in rotated_faces:
            x = np.array([vertex[0] for vertex in face])
            y = np.array([vertex[1] for vertex in face])
            z = np.array([vertex[2] for vertex in face])
            
            x = np.append(x, x[0])
            y = np.append(y, y[0])
            z = np.append(z, z[0])
            
            x = x.reshape((1, -1))
            y = y.reshape((1, -1))
            z = z.reshape((1, -1))
            
            self.ax.plot_wireframe(x, y, z, color='blue')

        origin = np.array([0, 0, 0])  # origin
        local_x = R @ np.array([1.5, 0, 0])  # local x-axis
        local_y = R @ np.array([0, 1.5, 0])  # local y-axis
        local_z = R @ np.array([0, 0, 1.5])  # local z-axis

        self.ax.quiver(*origin, *local_x, color='red', label='Local X', arrow_length_ratio=0.1)
        self.ax.quiver(*origin, *local_y, color='green', label='Local Y', arrow_length_ratio=0.1)
        self.ax.quiver(*origin, *local_z, color='blue', label='Local Z', arrow_length_ratio=0.1)

        self.ax.set_title(f"Quaternion: {np.round(q_current, 3)}")

    def run(self):
        self.animation = FuncAnimation(self.fig, self.update, interval=50, cache_frame_data=False)
        plt.show()
