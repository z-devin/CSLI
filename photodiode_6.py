from gpiozero import MCP3008
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import product, combinations

#      diode0 = +x
#      diode1 = +z
#      diode2 = -y
#      diode3 = +y
#      diode4 = -x
#      diode5 = -z
DIRECTIONS = np.array([
    [ 1,  0,  0],  # diode0, side1
    [ 0,  0,  1],  # diode1, side2
    [ 0, -1,  0],  # diode2, side3
    [ 0,  1,  0],  # diode3, side4
    [-1,  0,  0],  # diode4, side5
    [ 0,  0, -1],  # diode5, side6
])

def get_light_vector():
    diode0 = MCP3008(0).value
    diode1 = MCP3008(1).value
    diode2 = MCP3008(2).value
    diode3 = MCP3008(3).value
    diode4 = MCP3008(4).value
    diode5 = MCP3008(5).value

    intensities = np.array([diode0, diode1, diode2, diode3, diode4, diode5])

    #idx_top3" will be the indices of the top 3 diodes in descending order
    idx_top3 = np.argsort(intensities)[-3:]  # last 3 of sorted array = top 3

    #v = (I0 * dir0) + (I1 * dir1) + (I2 * dir2)
    v = np.zeros(3)
    for i in idx_top3:
        v += intensities[i] * DIRECTIONS[i]

    mag = np.linalg.norm(v) + 1e-12  # small epsilon to avoid zero-div
    sun_vec = v / mag

    return sun_vec, intensities, mag

def draw_cube(ax, edge=0.5):
    r = [-edge, edge]
    corners = np.array(list(product(r, r, r)))
    for s, e in combinations(corners, 2):
        if np.sum(np.abs(s - e)) == edge*2:
            ax.plot3D(*zip(s, e), color='blue')

def visualize_vector(initial_vec, initial_int, initial_mag):
    plt.ion()
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    draw_cube(ax, edge=0.5)

    quiv = ax.quiver(
        0, 0, 0,
        initial_vec[0], initial_vec[1], initial_vec[2],
        color='red',
        arrow_length_ratio=0.1
    )

    quiv0 = ax.quiver(
        0.5, 0, 0,
        initial_int[0]/initial_mag, 0, 0,
        color='green',
        arrow_length_ratio=0.1
    )

    quiv1 = ax.quiver(
        0, 0, 0.5,
        0, 0, initial_int[1]/initial_mag,
        color='green',
        arrow_length_ratio=0.1
    )

    quiv2 = ax.quiver(
        0, -0.5, 0,
        0, -initial_int[2]/initial_mag, 0,
        color='green',
        arrow_length_ratio=0.1
    )

    quiv3 = ax.quiver(
        0, 0.5, 0,
        0, initial_int[3]/initial_mag, 0,
        color='green',
        arrow_length_ratio=0.1
    )

    quiv4 = ax.quiver(
        -0.5, 0, 0,
        -initial_int[4]/initial_mag, 0, 0,
        color='green',
        arrow_length_ratio=0.1
    )

    quiv5 = ax.quiver(
        0, 0, -0.5,
        0, 0, -initial_int[5]/initial_mag,
        color='green',
        arrow_length_ratio=0.1
    )

    plt.title("Sun Vector (Top 3 Diodes)")
    plt.show()

    #loop
    try:
        while True:
            sun_vector, intensities, magnitude = get_light_vector()

            quiv.remove()
            quiv0.remove()
            quiv1.remove()
            quiv2.remove()
            quiv3.remove()
            quiv4.remove()
            quiv5.remove()

            quiv = ax.quiver(
                0, 0, 0,
                sun_vector[0], sun_vector[1], sun_vector[2],
                color='red',
                arrow_length_ratio=0.1
            )
            quiv0 = ax.quiver(
                0.5, 0, 0,
                intensities[0]/magnitude, 0, 0,
                color='green',
                arrow_length_ratio=0.1
            )
            quiv1 = ax.quiver(
                0, 0, 0.5,
                0, 0, intensities[1]/magnitude,
                color='green',
                arrow_length_ratio=0.1
            )
            quiv2 = ax.quiver(
                0, -0.5, 0,
                0, -intensities[2]/magnitude, 0,
                color='green',
                arrow_length_ratio=0.1
            )
            quiv3 = ax.quiver(
                0, 0.5, 0,
                0, intensities[3]/magnitude, 0,
                color='green',
                arrow_length_ratio=0.1
            )
            quiv4 = ax.quiver(
                -0.5, 0, 0,
                -intensities[4]/magnitude, 0, 0,
                color='green',
                arrow_length_ratio=0.1
            )
            quiv5 = ax.quiver(
                0, 0, -0.5,
                0, 0, -intensities[5]/magnitude,
                color='green',
                arrow_length_ratio=0.1
            )
            plt.draw()
            plt.pause(0.001)
            
    except KeyboardInterrupt:
        plt.close(fig)

init_light_vector, init_intensities , init_magnitude = get_light_vector() 
visualize_vector(init_light_vector, init_intensities, init_magnitude)