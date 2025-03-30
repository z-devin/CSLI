from gpiozero import MCP3008
from time import sleep
import time
import numpy as np
import matplotlib.pyplot as plt

#      diode0 = +x
#      diode1 = +z
#      diode2 = -y
#      diode3 = +y
#      diode4 = -x
#      diode5 = -z

def get_intensities():
    diode0 = MCP3008(0).value
    diode1 = MCP3008(1).value
    diode2 = MCP3008(2).value
    diode3 = MCP3008(3).value
    diode4 = MCP3008(4).value
    diode5 = MCP3008(5).value

    array = np.array([diode0, diode1, diode2, diode3, diode4, diode5])
    print(array)
    return array

def live_plot_intensities():
    plt.ion() 
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_ylim(0, 1.1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Intensity")
    ax.set_title("Live Photodiode Intensities")

    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    lines = []
    for i in range(6):
        line, = ax.plot([], [], label=f"Diode {i}", color=colors[i])
        lines.append(line)
    ax.legend(loc='upper right')


    max_points = 100
    times = []
    intensities_data = [[] for _ in range(6)]
    start_time = time.time()

    try:
        while True:
            current_time = time.time() - start_time
            intensities = get_intensities()
            times.append(current_time)
            for i in range(6):
                intensities_data[i].append(intensities[i])

            # Trim
            if len(times) > max_points:
                times = times[-max_points:]
                for i in range(6):
                    intensities_data[i] = intensities_data[i][-max_points:]

            # Update
            for i, line in enumerate(lines):
                line.set_data(times, intensities_data[i])

            # Update x-axis
            ax.set_xlim(times[0], times[-1])
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.0001)
    except KeyboardInterrupt:
        plt.close(fig)

live_plot_intensities()