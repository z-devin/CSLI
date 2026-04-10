from time import sleep
import numpy as np
import csv
from smbus2 import SMBus
import lgpio
from time import sleep, time

address = 12

h = lgpio.gpiochip_open(0)
DRDY_pin = 4
RSTN_pin = 17
lgpio.gpio_claim_input(h, DRDY_pin)
lgpio.gpio_claim_output(h, RSTN_pin, level=1)

def wait_for_mag_ready():
    start_time = time()
    while not lgpio.gpio_read(h, DRDY_pin):
        if time() - start_time > 0.3:
            raise TimeoutError("DRDY timeout")

def measure2dec(data):
    def ttod(bin_str):
        temp_int = int(bin_str[1:], base=2)
        temp_str_2 = '0' * (len(bin_str) - 1)
        temp_str_2 = bin_str[0] + temp_str_2
        temp_int_2 = int(temp_str_2, base=2)
        return -temp_int_2 + temp_int
    bin_str = bin(data[2])[8:] + bin(data[1])[2:].zfill(8) + bin(data[0])[2:].zfill(8)
    return ttod(bin_str)

def magnetometer_setup(bus):
    bus.write_byte_data(address, 0x32, 0b00000000)
    bus.write_byte_data(address, 0x32, 0b00001000)

def reset_magnetometer(bus):
    lgpio.gpio_write(h, RSTN_pin, 0)
    sleep(0.05)
    lgpio.gpio_write(h, RSTN_pin, 1)
    magnetometer_setup(bus)

def read_mag_data(bus):
    data = bus.read_i2c_block_data(address, 0x11, 9)
    st2 = bus.read_byte_data(address, 0x1B)

    x_data = measure2dec(data[0:3])
    y_data = measure2dec(data[3:6])
    z_data = measure2dec(data[6:9])

    d = np.array([x_data, y_data, z_data])
    A = np.array([[1.0274, -0.0058, 0.0192],
                  [-0.0058, 1.0302, 0.0149],
                  [0.0192, 0.0149, 0.9455]])
    b = np.array([2.4272e3, -1.0466e4, -1.8667e4])
    scale_factors = np.array([10.08706565, 9.95520947, 10.33009517])

    return (d - b) @ A * scale_factors

def read_magnetometer(bus):
    try:
        wait_for_mag_ready()
        return read_mag_data(bus)
    except TimeoutError:
        bus.read_byte_data(address, 0x1B)
        return np.array([0.0, 0.0, 0.0])
    except Exception as e:
        bus.read_byte_data(address, 0x1B)
        raise e

if __name__ == "__main__":
    with SMBus(1) as bus:
        magnetometer_setup(bus)

        with open('mag_data_calibrate.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])

            while True:
                new_vec = read_magnetometer(bus)
                writer.writerow(new_vec)
                f.flush()
                print(new_vec)

    lgpio.gpio_free(h, DRDY_pin)
    lgpio.gpio_free(h, RSTN_pin)
    lgpio.gpiochip_close(h)