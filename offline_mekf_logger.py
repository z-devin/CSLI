import numpy as np
from photodiode_6 import get_light_vector
from magnetometer_visual_updated import wait_for_mag_ready, read_mag_data, magnetometer_setup
from scipy.spatial.transform import Rotation
from smbus2 import SMBus
import time
import csv
import warnings
warnings.filterwarnings("ignore")

# TRIAD math
def normalize(vec):
    return vec / np.linalg.norm(vec)

def triad(b1, b2, r1, r2):
    b1 = normalize(b1)
    b2 = normalize(b2)
    r1 = normalize(r1)
    r2 = normalize(r2)
    t1 = b1
    t2 = normalize(np.cross(b1, b2))
    t3 = np.cross(t1, t2)
    R1 = r1
    R2 = normalize(np.cross(r1, r2))
    R3 = np.cross(R1, R2)
    T_body = np.column_stack((t1, t2, t3))
    T_inertial = np.column_stack((R1, R2, R3))
    return T_body @ T_inertial.T

# Single bus
bus = SMBus(1)

# Initialize magnetometer
magnetometer_setup(bus)
time.sleep(0.1)

# IMU config
IMU_ADDR = 0x68
SMPLRT_DIV = 7
HARDWARE_DT = 1.0 / (1000.0 / (1 + SMPLRT_DIV))

# IMU init
bus.write_byte_data(IMU_ADDR, 0x6B, 1)
bus.write_byte_data(IMU_ADDR, 0x19, SMPLRT_DIV)
bus.write_byte_data(IMU_ADDR, 0x1A, 2)
bus.write_byte_data(IMU_ADDR, 0x1B, 0)  # ±250°/s
bus.write_byte_data(IMU_ADDR, 0x38, 1)

################### Set reference vectors from initial readings ###################
print("Reading initial sensor values for reference vectors...")
time.sleep(0.5)

sun_samples = []
mag_samples = []
for _ in range(50):
    try:
        sv, _, _ = get_light_vector()
        sun_samples.append(sv)
    except:
        pass
    try:
        wait_for_mag_ready()
        mv = read_mag_data(bus)
        mag_samples.append(mv)
    except:
        pass
    time.sleep(0.02)

r1_inertial = normalize(np.mean(sun_samples, axis=0))
r2_inertial = normalize(np.mean(mag_samples, axis=0))

print(f"r1_inertial (sun): [{r1_inertial[0]:.4f}, {r1_inertial[1]:.4f}, {r1_inertial[2]:.4f}]")
print(f"r2_inertial (mag): [{r2_inertial[0]:.4f}, {r2_inertial[1]:.4f}, {r2_inertial[2]:.4f}]")


def read_gyro_direct():
    """Read gyro directly from registers (not FIFO)"""
    raw = bus.read_i2c_block_data(IMU_ADDR, 0x43, 6)
    gx_raw = (raw[0] << 8) | raw[1]
    gy_raw = (raw[2] << 8) | raw[3]
    gz_raw = (raw[4] << 8) | raw[5]
    if gx_raw > 32768: gx_raw -= 65536
    if gy_raw > 32768: gy_raw -= 65536
    if gz_raw > 32768: gz_raw -= 65536
    return np.radians(np.array([-gx_raw / 131.0, -gy_raw / 131.0, gz_raw / 131.0]))


def drain_fifo():
    """Read all available samples from MPU6050 FIFO"""
    fifo_count_h = bus.read_byte_data(IMU_ADDR, 0x72)
    fifo_count_l = bus.read_byte_data(IMU_ADDR, 0x73)
    fifo_count = (fifo_count_h << 8) | fifo_count_l

    bytes_per_sample = 6
    num_samples = fifo_count // bytes_per_sample
    samples = []

    for _ in range(num_samples):
        raw = bus.read_i2c_block_data(IMU_ADDR, 0x74, bytes_per_sample)

        gx_raw = (raw[0] << 8) | raw[1]
        gy_raw = (raw[2] << 8) | raw[3]
        gz_raw = (raw[4] << 8) | raw[5]

        if gx_raw > 32768: gx_raw -= 65536
        if gy_raw > 32768: gy_raw -= 65536
        if gz_raw > 32768: gz_raw -= 65536

        gx = -gx_raw / 131.0
        gy = -gy_raw / 131.0
        gz = gz_raw / 131.0

        samples.append(np.radians(np.array([gx, gy, gz])))

    return samples


################### Gyro Bias Calibration (direct reads, no FIFO) ###################
print("Keep CubeSat still - calibrating gyro bias...")
time.sleep(0.5)

cal_samples = []
for _ in range(250):
    cal_samples.append(read_gyro_direct())
    time.sleep(0.008)

gyro_bias = np.mean(cal_samples, axis=0)
gyro_bias_std = np.std(cal_samples, axis=0)

print(f"Gyro bias: [{np.degrees(gyro_bias[0]):.3f}, {np.degrees(gyro_bias[1]):.3f}, {np.degrees(gyro_bias[2]):.3f}] deg/s")
print(f"Gyro noise: [{np.degrees(gyro_bias_std[0]):.3f}, {np.degrees(gyro_bias_std[1]):.3f}, {np.degrees(gyro_bias_std[2]):.3f}] deg/s")
print(f"Calibration samples: {len(cal_samples)}")

with open('30deg_gyro_calibration.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['bias_gx', 'bias_gy', 'bias_gz', 'noise_gx', 'noise_gy', 'noise_gz'])
    writer.writerow([gyro_bias[0], gyro_bias[1], gyro_bias[2],
                     gyro_bias_std[0], gyro_bias_std[1], gyro_bias_std[2]])

print("Calibration done\n")

################### Enable FIFO (once, never reset again) ###################
bus.write_byte_data(IMU_ADDR, 0x6A, 0x04)  # reset FIFO
time.sleep(0.01)
bus.write_byte_data(IMU_ADDR, 0x23, 0x70)  # FIFO_EN: gyro XYZ
bus.write_byte_data(IMU_ADDR, 0x6A, 0x40)  # enable FIFO

# Flush and verify alignment
time.sleep(0.05)
drain_fifo()
time.sleep(0.05)

direct = read_gyro_direct()
fifo_samples = drain_fifo()
if fifo_samples:
    fifo_last = fifo_samples[-1]
    print(f"Direct: [{np.degrees(direct[0]):.1f}, {np.degrees(direct[1]):.1f}, {np.degrees(direct[2]):.1f}]")
    print(f"FIFO:   [{np.degrees(fifo_last[0]):.1f}, {np.degrees(fifo_last[1]):.1f}, {np.degrees(fifo_last[2]):.1f}]")

    # Verify alignment
    diff = np.abs(np.degrees(direct) - np.degrees(fifo_last))
    if np.any(diff > 1.0):
        print("WARNING: FIFO and direct read don't match! Axes may be misaligned.")
    else:
        print("FIFO alignment verified OK")

# Quick bias check
print("\nBias-corrected check (should be near zero):")
time.sleep(0.1)
check = drain_fifo()
for s in check[:5]:
    corrected = s - gyro_bias
    print(f"  [{np.degrees(corrected[0]):.3f}, {np.degrees(corrected[1]):.3f}, {np.degrees(corrected[2]):.3f}] deg/s")

################### Open Log Files ###################
imu_file = open('30deg_imu_log.csv', 'w', newline='')
triad_file = open('30deg_triad_log.csv', 'w', newline='')
mag_file = open('30deg_mag_log.csv', 'w', newline='')
sun_file = open('30deg_sun_log.csv', 'w', newline='')

imu_writer = csv.writer(imu_file)
triad_writer = csv.writer(triad_file)
mag_writer = csv.writer(mag_file)
sun_writer = csv.writer(sun_file)

imu_writer.writerow(['timestamp', 'gx', 'gy', 'gz'])
triad_writer.writerow(['timestamp', 'qw', 'qx', 'qy', 'qz'])
mag_writer.writerow(['timestamp', 'mx', 'my', 'mz'])
sun_writer.writerow(['timestamp', 'sx', 'sy', 'sz'])

################### Main Logging Loop ###################
print("\nLogging raw sensor data... Ctrl+C to stop")

try:
    while True:
        # 1. Drain IMU FIFO (bias-corrected)
        samples = drain_fifo()
        drain_time = time.monotonic()
        n = len(samples)

        for i, omega in enumerate(samples):
            corrected = omega - gyro_bias
            sample_time = drain_time - (n - 1 - i) * HARDWARE_DT
            imu_writer.writerow([sample_time, corrected[0], corrected[1], corrected[2]])
        imu_file.flush()

        # 2. Sun sensor
        try:
            b1_body, _, _ = get_light_vector()
            sun_writer.writerow([time.monotonic(), b1_body[0], b1_body[1], b1_body[2]])
            sun_file.flush()
        except Exception as e:
            print(f"Sun sensor error: {e}")
            b1_body = np.zeros(3)

        # 3. Magnetometer
        try:
            wait_for_mag_ready()
            b2_body = read_mag_data(bus)
            mag_writer.writerow([time.monotonic(), b2_body[0], b2_body[1], b2_body[2]])
            mag_file.flush()
        except TimeoutError:
            bus.read_byte_data(12, 0x1B)
            b2_body = np.zeros(3)
        except Exception as e:
            print(f"Mag error: {e}")
            b2_body = np.zeros(3)

        # 4. TRIAD
        try:
            if not np.all(b1_body == 0) and not np.all(b2_body == 0):
                if abs(np.dot(normalize(b1_body), normalize(b2_body))) < 0.99:
                    R_BI = triad(b1_body, b2_body, r1_inertial, r2_inertial)
                    if not np.any(np.isnan(R_BI)):
                        q_scipy = Rotation.from_matrix(R_BI.T).as_quat()
                        q_meas = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])
                        triad_writer.writerow([time.monotonic(), q_meas[0], q_meas[1], q_meas[2], q_meas[3]])
                        triad_file.flush()
        except Exception as e:
            print(f"TRIAD error: {e}")

except KeyboardInterrupt:
    imu_file.close()
    triad_file.close()
    mag_file.close()
    sun_file.close()
    bus.close()
    print("Done - files saved")