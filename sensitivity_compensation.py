from smbus2 import SMBus
import lgpio
from time import sleep
import numpy as np

ADDRESS = 12
DRDY_PIN = 4
RSTN_PIN = 17
BSE = 10.0  # nT/LSB

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(h, DRDY_PIN)
lgpio.gpio_claim_output(h, RSTN_PIN, level=1)


def wait_drdy():
    from time import time
    start = time()
    while not lgpio.gpio_read(h, DRDY_PIN):
        if time() - start > 0.3:
            raise TimeoutError("DRDY timeout")


def to_int18(data):
    raw = (data[2] << 16) | (data[1] << 8) | data[0]
    raw &= 0x3FFFF
    return raw - 0x40000 if raw & 0x20000 else raw


def to_int11(data):
    raw = ((data[1] & 0x07) << 8) | data[0]
    return raw - 0x800 if raw & 0x400 else raw


def to_int12(data):
    raw = ((data[1] & 0x0F) << 8) | data[0]
    return raw - 0x1000 if raw & 0x800 else raw


def self_test(bus):
    bus.write_byte_data(ADDRESS, 0x32, 0x00)  # power down
    sleep(0.001)
    bus.write_byte_data(ADDRESS, 0x32, 0x70)  # LN drive 2 + self-test
    wait_drdy()
    data = bus.read_i2c_block_data(ADDRESS, 0x11, 9)
    bus.read_byte_data(ADDRESS, 0x1B)          # read ST2 to release
    return np.array([to_int18(data[0:3]), to_int18(data[3:6]), to_int18(data[6:9])], dtype=float)


def compute_scale_factors(bus, averages=4):
    # Trigger single measurement so reference registers populate
    bus.write_byte_data(ADDRESS, 0x32, 0x01)
    wait_drdy()
    bus.read_byte_data(ADDRESS, 0x1B)

    ref_data = bus.read_i2c_block_data(ADDRESS, 0x20, 6)
    reference = np.array([to_int11(ref_data[0:2]), to_int11(ref_data[2:4]), to_int12(ref_data[4:6])], dtype=float)

    current = np.mean([self_test(bus) for _ in range(averages)], axis=0)

    return BSE * (reference / current)


if __name__ == "__main__":
    with SMBus(1) as bus:
        scale_factors = compute_scale_factors(bus,averages=100)
        print(f"Scale factors (nT/LSB): {scale_factors}")

    lgpio.gpio_free(h, DRDY_PIN)
    lgpio.gpio_free(h, RSTN_PIN)
    lgpio.gpiochip_close(h)