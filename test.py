from smbus2 import SMBus
from magnetometer_visual_updated import magnetometer_setup
import time
import numpy as np

bus = SMBus(1)
magnetometer_setup(bus)
time.sleep(0.1)

IMU_ADDR = 0x68
bus.write_byte_data(IMU_ADDR, 0x6B, 1)
bus.write_byte_data(IMU_ADDR, 0x1B, 0)

def read_gyro():
    raw = bus.read_i2c_block_data(IMU_ADDR, 0x43, 6)
    gx = (raw[0] << 8) | raw[1]
    gy = (raw[2] << 8) | raw[3]
    gz = (raw[4] << 8) | raw[5]
    if gx > 32768: gx -= 65536
    if gy > 32768: gy -= 65536
    if gz > 32768: gz -= 65536
    return gx / 131.0, gy / 131.0, gz / 131.0

# Calibrate bias
print("Keep still - calibrating...")
cal = []
for _ in range(200):
    cal.append(read_gyro())
    time.sleep(0.008)
bias = np.mean(cal, axis=0)
print(f"Bias: gx={bias[0]:+.1f}  gy={bias[1]:+.1f}  gz={bias[2]:+.1f}")
print(f"\nNow rotate one axis at a time (right-hand rule):")
print(f"  +X rotation → gx should go positive")
print(f"  +Y rotation → gy should go positive")
print(f"  +Z rotation → gz should go positive\n")

try:
    while True:
        gx, gy, gz = read_gyro()
        gx -= bias[0]
        gy -= bias[1]
        gz -= bias[2]
        
        # Highlight which axis is active
        labels = ['  ', '  ', '  ']
        vals = [gx, gy, gz]
        max_idx = np.argmax(np.abs(vals))
        if abs(vals[max_idx]) > 5.0:
            labels[max_idx] = '<<'
        
        print(f"gx: {gx:+7.1f} {labels[0]} gy: {gy:+7.1f} {labels[1]} gz: {gz:+7.1f} {labels[2]} deg/s")
        time.sleep(0.1)
except KeyboardInterrupt:
    bus.close()