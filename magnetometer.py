from smbus2 import SMBus
import RPi.GPIO as GPIO
from time import sleep
import numpy as np

address = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.IN)

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

def factory2dec(data, z = False):
    if z:
        bin_str = bin(data[1])[6:] + bin(data[0])[2:].zfill(8)
    else:
        bin_str = bin(data[1])[7:] + bin(data[0])[2:].zfill(8)
    return(ttod(bin_str))

def read_factory():
    with SMBus(1) as bus:
        factory_data = bus.read_i2c_block_data(address, 0x20, 6)    # Read factory data

    print(factory_data)
    x_factory = factory2dec(factory_data[0:2])
    y_factory = factory2dec(factory_data[2:4])
    z_factory = factory2dec(factory_data[4:6], z=True)

    print(f"X Factory: ", x_factory)
    print(f"Y Factory: ", y_factory)
    print(f"Z Factory: ", z_factory)
    print(f"Factory Magnitude: {np.linalg.norm([x_factory, y_factory, z_factory])}")

with SMBus(1) as bus:
    bus.write_byte_data(address, 0x32, 0b00000000)  # Set power down mode
    bus.write_byte_data(address, 0x32, 0b01100000)  # Set Low Noise Drive 2
    bus.write_byte_data(address, 0x32, 0b01110000)  # Self-test mode
    bus.write_byte_data(address, 0x32, 0b01100001)  # Low noise single-measurement mode
    # bus.write_byte_data(address, 0x32, 0b00000001)  # Single-measurement mode

    try:
        while not GPIO.input(7):    # Check for data ready
            pass
        # print(GPIO.input(7))

        data = bus.read_i2c_block_data(address, 0x11, 9)    # Read data
        # # data = bus.read_i2c_block_data(address, 0x0, 20)    # Read data


        x_data = measure2dec(data[0:3])
        y_data = measure2dec(data[3:6])
        z_data = measure2dec(data[6:9])

        d = np.array([x_data, y_data, z_data])
        A = np.array([[0.9413, -0.0043, 0.0376],
                    [-0.0043, 0.9909, 0.0086],
                    [0.0376, 0.0086, 1.0737]])

        b = np.array([3.3744e3, -7.7696e3, -1.6720e4])

        out=(d-b)@A
        """
        print(f"X: ", x_data)
        print(f"Y: ", y_data)
        print(f"Z: ", z_data)
        print(f"Magnitude: {np.linalg.norm([x_data, y_data, z_data])}")"""

        print(f"Output: {out}")

        st2 = bus.read_byte_data(address, 0x1B)     # Read ST2 required
        # print(st2)
    except:
        st2 = bus.read_byte_data(address, 0x1B)     # Read ST2 required
        # print("st2 read")


GPIO.cleanup()
# 0b1011
# 0x456



