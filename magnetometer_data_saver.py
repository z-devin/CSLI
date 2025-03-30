from gpiozero import MCP3008
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import product, combinations

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

def magnetometer_setup(bus):
    bus.write_byte_data(address, 0x32, 0b00000000)  # Set power down mode
    # bus.write_byte_data(address, 0x32, 0b01100000)  # Set Low Noise Drive 2
    # bus.write_byte_data(address, 0x32, 0b00010000)  # Self-test mode
    bus.write_byte_data(address, 0x32, 0b00001000)  # 100 Hz continuous measurement mode

def read_magnetometer():
    try:
        while not GPIO.input(7):    # Check for data ready
            pass
        # print(GPIO.input(7))

        data = bus.read_i2c_block_data(address, 0x11, 9)    # Read data
        


        x_data = measure2dec(data[0:3])#*x_factor# + x_offset
        y_data = measure2dec(data[3:6])#*y_factor# + y_offset
        z_data = measure2dec(data[6:9])#*z_factor# + z_offset

        st2 = bus.read_byte_data(address, 0x1B)     # Read ST2 required
    except:
        st2 = bus.read_byte_data(address, 0x1B)     # Read ST2 required
        # print("st2 read")
    return [x_data, y_data, z_data]

with SMBus(1) as bus:
    magnetometer_setup(bus)
    data = np.array(read_magnetometer())
    while True:
        try:
            new_vec = np.array(read_magnetometer())
            print(new_vec)
            data = np.vstack([data, new_vec])
        except Exception as e:
            f = open("data2.csv", "ab")
            np.savetxt(f, data, delimiter=",")
            f.close()
            raise e
            # break
        
        

GPIO.cleanup()
# 0b1011
# 0x456



