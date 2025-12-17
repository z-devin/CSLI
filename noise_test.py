from gpiozero import MCP3008
from gpiozero.pins.native import NativeFactory
from time import sleep
import time
import numpy as np
import matplotlib.pyplot as plt

# Create a single factory instance
factory = NativeFactory()

# Create ADC objects once at startup
diode_channels = []
for i in range(6):
    diode_channels.append(MCP3008(channel=i, pin_factory=factory))

def get_intensities():
    # Read values from existing ADC objects
    array = np.array([
        diode_channels[0].value * 3.3,
        diode_channels[1].value * 3.3,
        diode_channels[2].value * 3.3,
        diode_channels[3].value * 3.3,
        diode_channels[4].value * 3.3,
        diode_channels[5].value * 3.3
    ])
    print(array)
    return array

def live_plot_intensities():
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_ylim(0, 3.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Intensity")
    ax.set_title("Live Photodiode Intensities")
    
    colors = ['r', 'g', 'b', 'c', 'm', 'y']
    lines = []
    for i in range(6):
        line, = ax.plot([], [], label=f"Diode {i+1}", color=colors[i])
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
            
            # Add small delay to prevent CPU overload
            plt.pause(0.01)
            
    except KeyboardInterrupt:
        plt.close(fig)

live_plot_intensities()