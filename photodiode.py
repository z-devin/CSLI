from gpiozero import MCP3008
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import product, combinations


def get_light_vector():
    diode0 = MCP3008(3)
    diode1 = MCP3008(1)
    diode2 = MCP3008(4)
    I_x = diode0.value
    I_y = diode1.value
    I_z = diode2.value

    print(np.array([I_x,I_y,I_z]))

    mag = np.sqrt(I_x**2 + I_y**2 + I_z**2)

    L_x = I_x/mag
    L_y = I_y/mag
    L_z = I_z/mag

    return np.array([L_x, L_z, L_y])

def draw_cube(ax, edge=0.5):
    r = [-edge, edge]
    corners = np.array(list(product(r, r, r)))
    
    for s, e in combinations(corners, 2):
        if np.sum(np.abs(s - e)) ==  edge*2:
            ax.plot3D(*zip(s, e), color='blue')

def visualize_vector(vec):
    plt.ion()

    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    draw_cube(ax, edge=0.5)

    quiv = ax.quiver(
        0, 0, 0,  # tail
        vec[0], vec[1], vec[2], # tip
        color='red',
        arrow_length_ratio=0.1
    )

    quivx = ax.quiver(
        0, 0, 0,  # tail
        1, 0, 0, # tip
        color='blue',
        arrow_length_ratio=0.1
    )

    plt.title("Cube Visual")
    plt.show()

    while True:
        x_new, y_new, z_new = get_light_vector()
        vec = np.array([x_new, y_new, z_new])

        quiv.remove()
        quiv = ax.quiver(
            0, 0, 0,
            vec[0], vec[1], vec[2],
            color='red',
            arrow_length_ratio=0.1
        )

        plt.draw()
        plt.pause(0.01)

visualize_vector(get_light_vector())