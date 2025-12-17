import spidev
import time
import numpy as np
import matplotlib.pyplot as plt

# Setup SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 500000  # Reduced to 500kHz for more stability
spi.mode = 0
spi.bits_per_word = 8  # Explicitly set bits per word

def read_adc(channel):
    """
    Read the specified ADC channel using hardware SPI
    """
    # Construct the command bytes for MCP3008
    cmd = [0x01, (0x08 + channel) << 4, 0x00]
    
    # Send command and get response
    resp = spi.xfer2(cmd)
    
    # Extract the 10-bit value
    adc_value = ((resp[1] & 0x03) << 8) + resp[2]
    
    # Convert to voltage (0-3.3V)
    voltage = adc_value * 3.3 / 1023.0
    
    return voltage

def get_intensities():
    # Read values from all 6 channels
    array = np.array([
        read_adc(0),
        read_adc(1),
        read_adc(2),
        read_adc(3),
        read_adc(4),
        read_adc(5)
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
            
            # Small delay before reading to stabilize SPI bus
            time.sleep(0.001)
            intensities = get_intensities()
            times.append(current_time)
            
            for i in range(6):
                intensities_data[i].append(intensities[i])
            
            # Trim
            if len(times) > max_points:
                times = times[-max_points:]
                for i in range(6):
                    intensities_data[i] = intensities_data[i][-max_points:]
            
            # Update plot
            for i, line in enumerate(lines):
                line.set_data(times, intensities_data[i])
            
            # Update x-axis
            if times:
                ax.set_xlim(times[0], times[-1])
            
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            # Longer pause to prevent resource contention
            plt.pause(0.1)
            
    except KeyboardInterrupt:
        plt.close(fig)
        spi.close()  # Clean up SPI
    except Exception as e:
        print(f"Error: {e}")
        plt.close(fig)
        spi.close()  # Clean up SPI on error

if __name__ == "__main__":
    try:
        print("Starting photodiode monitoring with hardware SPI...")
        print("Press Ctrl+C to exit")
        live_plot_intensities()
    finally:
        try:
            spi.close()  # Ensure SPI is closed even on errors
            print("SPI closed.")
        except:
            pass