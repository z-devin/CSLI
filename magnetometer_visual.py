from gpiozero import MCP3008
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import product, combinations
import csv

def draw_cube(ax, edge=0.5):
    r = [-edge, edge]
    corners = np.array(list(product(r, r, r)))
    
    for s, e in combinations(corners, 2):
        if np.sum(np.abs(s - e)) ==  edge*2:
            ax.plot3D(*zip(s, e), color='blue')

def graphics_setup():
    plt.ion()

    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    draw_cube(ax, edge=0.5)
    quiv = ax.quiver(
        0, 0, 0,  # tail
        0.1, 0.1, 0.1, # tip
        color='red',
        arrow_length_ratio=0.1
    )
    quivx = ax.quiver(
        0, 0, 0,  # tail
        1, 0, 0, # tip
        color='blue',
        arrow_length_ratio=0.1
    )
    quivy = ax.quiver(
        0, 0, 0,  # tail
        0, 1, 0, # tip
        color='green',
        arrow_length_ratio=0.1
    )
    quivz = ax.quiver(
        0, 0, 0,  # tail
        0, 0, 1, # tip
        color='yellow',
        arrow_length_ratio=0.1
    )
    plt.title("Cube Visual")
    plt.show()
    return quiv, ax


def visualize_vector(new_vec, quiv, ax):    
    x_new, y_new, z_new = new_vec
    # vec = np.array([x_new, y_new, z_new])

    quiv.remove()
    quiv = ax.quiver(
        0, 0, 0,
        x_new, y_new, z_new,
        color='red',
        arrow_length_ratio=0.1
    )

    #quiv.set_segments([[0, 0, 0], [x_new, y_new, z_new]])

    plt.draw()
    plt.pause(0.011)
    return quiv

from smbus2 import SMBus
# import RPi.GPIO as GPIO
import lgpio
from time import sleep, time
import numpy as np

address = 12

# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(7, GPIO.IN)

h = lgpio.gpiochip_open(0)
DRDY_pin = 4
RSTN_pin = 17
lgpio.gpio_claim_input(h, DRDY_pin)
lgpio.gpio_claim_output(h, RSTN_pin, level=1)  # start high

#DRDY_read = GPIO.input(7)
#print(DRDY_read)

# To fix GPIO permissions errors run:
# sudo chown cubesat /dev/gpiomem
# sudo chmod g+rw /dev/gpiomem

def ttod(bin_str): # twos complement to decimal    
    temp_int = int(bin_str[1:], base=2)

    temp_str_2 = '0'*(len(bin_str)-1)
    temp_str_2 = bin_str[0]+ temp_str_2
    temp_int_2 = int(temp_str_2, base=2)
    output = -temp_int_2+temp_int

    return output

def measure2dec(data):
    bin_str = bin(data[2])[8:] + bin(data[1])[2:].zfill(8) + bin(data[0])[2:].zfill(8)
    return(ttod(bin_str))

def magnetometer_setup(bus):
    bus.write_byte_data(address, 0x32, 0b00000000)  # Set power down mode
    # bus.write_byte_data(address, 0x32, 0b01100000)  # Set Low Noise Drive 2
    # bus.write_byte_data(address, 0x32, 0b00010000)  # Self-test mode
    bus.write_byte_data(address, 0x32, 0b00001000)  # continuos reading mode (100 Hz)

def reset_magnetometer(bus):
    lgpio.gpio_write(h, RSTN_pin, 0)
    sleep(0.05)  # TODO: this is probably not the best way to do this, will freeze entire program
    lgpio.gpio_write(h, RSTN_pin, 1)
    magnetometer_setup(bus)

def read_magnetometer(bus):
    try:
        # while not GPIO.input(7):    # Check for data ready
        start_time = time()
        while not lgpio.gpio_read(h, DRDY_pin):
            if time() - start_time > 0.3:
                reset_magnetometer(bus)
                raise TimeoutError
        # print(GPIO.input(7))

        data = bus.read_i2c_block_data(address, 0x11, 9)    # Read data
        # # data = bus.read_i2c_block_data(address, 0x0, 20)    # Read data
        # print(data)

        x_data = measure2dec(data[0:3])
        y_data = measure2dec(data[3:6])
        z_data = measure2dec(data[6:9])

        d = np.array([x_data, y_data, z_data])
        A = np.array([[0.9318, -0.0353, -0.0041],
                      [-0.0353, 0.9532, 0.0912],
                      [-0.0041, 0.0912, 1.1362]])
        b = np.array([3.274e3, -7.9187e3, -1.6536e4])

        out=(d-b)@A

        """print(f"X: ", x_data)
        print(f"Y: ", y_data)
        print(f"Z: ", z_data)
        print(f"Magnitude: {np.linalg.norm([x_data, y_data, z_data])}")"""
        # print(f"Magnetometer: {out}")

        st2 = bus.read_byte_data(address, 0x1B)     # Read ST2 required
    except TimeoutError:
        st2 = bus.read_byte_data(address, 0x1B)     # Read ST2 required
        return read_magnetometer(bus)  # TODO: this may be bad, could lead to getting stuck in infinite loop 
    except Exception as e:
        st2 = bus.read_byte_data(address, 0x1B)     # Read ST2 required
        raise e
        # print("st2 read")
    return out
if __name__ == "__main__":

    with SMBus(1) as bus:
        magnetometer_setup(bus)
        quiv, ax = graphics_setup()

        with open('mag_data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])

            while True:
                new_vec = read_magnetometer(bus)
                new_vec /= np.linalg.norm(new_vec)
                writer.writerow(new_vec)
                f.flush()
                print(new_vec)
                quiv = visualize_vector(new_vec, quiv, ax)

#GPIO.cleanup()
lgpio.gpio_free(h, DRDY_pin)
lgpio.gpio_free(h, RSTN_pin)
lgpio.gpiochip_close(h)
# 0b1011
# 0x456



