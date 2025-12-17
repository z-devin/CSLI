import numpy as np
from imu import get_imu_data, MPU_Init
from triad_function import get_triad_data
import threading
import time
import csv

################### User Configuration ###################
# IMU noise characteristics
gyro_noise_std_x = 0.006906
gyro_noise_std_y = 0.013501
gyro_noise_std_z = 0.004708

bias_drift_rate_x = 1.501764e-07
bias_drift_rate_y = -6.737916e-08
bias_drift_rate_z = 2.869799e-09

gyro_noise_cov = np.diag((gyro_noise_std_x**2, gyro_noise_std_y**2, gyro_noise_std_z**2))
bias_drift_cov = np.diag((bias_drift_rate_x**2, bias_drift_rate_y**2, bias_drift_rate_z**2))

sampling_rate = 100  # IMU sampling rate [Hz]

# TRIAD measurement noise
R = np.diag([1.8190332587e-06, 1.5939245208e-05, 1.9316665858e-08])

# Complementary filter parameter
alpha = 0.95

# MEKF initial covariance
P_attitude_init = np.radians(10.0)**2
P_bias_init = np.radians(0.5)**2

################### Global Variables ###################
dt = 1.0 / sampling_rate

# Thread-safe data containers
class SensorData:
    def __init__(self):
        self.lock = threading.Lock()
        self.omega = np.zeros(3)
        self.triad_available = False
        self.q_triad = np.array([1.0, 0.0, 0.0, 0.0])
        self.new_triad_data = False

class FilterState:
    def __init__(self):
        self.lock = threading.Lock()
        self.q_mekf = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.bias_est_mekf = np.zeros(3)
        self.current_time = 0.0
        self.triad_active = False

sensor_data = SensorData()
filter_state = FilterState()
running = threading.Event()
running.set()

################### Sensor Threads ###################
def imu_thread():
    """Continuously read IMU data at high rate"""
    print("IMU thread started")
    while running.is_set():
        try:
            omega = get_imu_data()
            with sensor_data.lock:
                sensor_data.omega = omega
        except Exception as e:
            print(f"IMU error: {e}")
        time.sleep(dt)

def triad_thread():
    """Continuously read TRIAD data (slower, event-driven)"""
    print("TRIAD thread started")
    while running.is_set():
        try:
            available, q_meas = get_triad_data()
            if available:
                with sensor_data.lock:
                    sensor_data.triad_available = True
                    sensor_data.q_triad = q_meas
                    sensor_data.new_triad_data = True
        except Exception as e:
            print(f"TRIAD error: {e}")
        time.sleep(0.1)

################### Quaternion Functions ###################
def integrate_quaternion(q, omega, dt):
    omega_norm = np.linalg.norm(omega)
    if omega_norm < 1e-10:
        return q / np.linalg.norm(q)
    half_angle = 0.5 * omega_norm * dt
    s = np.sin(half_angle) / omega_norm
    q_delta = np.array([np.cos(half_angle), s*omega[0], s*omega[1], s*omega[2]])
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = q_delta
    q_new = np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])
    return q_new / np.linalg.norm(q_new)

################### Filter Thread ###################
def filter_thread(csv_writer, csv_file):
    """Main filtering loop running at 100 Hz"""
    print("Filter thread started")
    
    # Initialize state
    q_comp = np.array([1.0, 0.0, 0.0, 0.0])
    bias_est_comp = np.zeros(3)
    
    q_mekf = np.array([1.0, 0.0, 0.0, 0.0])
    bias_est_mekf = np.zeros(3)
    
    q_imu = np.array([1.0, 0.0, 0.0, 0.0])
    
    P = np.eye(6)
    P[0:3, 0:3] *= P_attitude_init
    P[3:6, 3:6] *= P_bias_init
    
    Q = np.block([
        [gyro_noise_cov, np.zeros((3,3))],
        [np.zeros((3,3)), bias_drift_cov]
    ])
    
    current_time = 0.0
    
    while running.is_set():
        # Get latest sensor data
        with sensor_data.lock:
            omega_measured = sensor_data.omega.copy()
            triad_available = sensor_data.new_triad_data
            q_triad_meas = sensor_data.q_triad.copy()
            sensor_data.new_triad_data = False

        ################### IMU-only integration ###################
        q_imu = integrate_quaternion(q_imu, omega_measured, dt)

        ################### MEKF ###################
        omega_corrected_mekf = omega_measured - bias_est_mekf
        q_mekf = integrate_quaternion(q_mekf, omega_corrected_mekf, dt)

        omega_cross = np.array([
            [0, -omega_corrected_mekf[2], omega_corrected_mekf[1]],
            [omega_corrected_mekf[2], 0, -omega_corrected_mekf[0]],
            [-omega_corrected_mekf[1], omega_corrected_mekf[0], 0]
        ])

        F = np.zeros((6, 6))
        F[0:3, 0:3] = np.eye(3) - omega_cross * dt
        F[0:3, 3:6] = -dt * np.eye(3)
        F[3:6, 3:6] = np.eye(3)

        P = F @ P @ F.T + Q

        if triad_available:
            q_meas = q_triad_meas
            q_ref_conj = np.array([q_mekf[0], -q_mekf[1], -q_mekf[2], -q_mekf[3]])
            w1, x1, y1, z1 = q_meas
            w2, x2, y2, z2 = q_ref_conj
            delta_q = np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            ])

            if delta_q[0] < 0:
                delta_q = -delta_q

            z = 2 * delta_q[1:]
            H = np.zeros((3, 6))
            H[0:3, 0:3] = np.eye(3)
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            delta_x = K @ z
            delta_theta = delta_x[0:3]
            delta_bias = delta_x[3:6]
            P = (np.eye(6) - K @ H) @ P

            delta_theta_norm = np.linalg.norm(delta_theta)
            if delta_theta_norm > 1e-10:
                delta_q_update = np.array([
                    np.cos(delta_theta_norm / 2),
                    (delta_theta[0] / delta_theta_norm) * np.sin(delta_theta_norm / 2),
                    (delta_theta[1] / delta_theta_norm) * np.sin(delta_theta_norm / 2),
                    (delta_theta[2] / delta_theta_norm) * np.sin(delta_theta_norm / 2)
                ])

                w1, x1, y1, z1 = delta_q_update
                w2, x2, y2, z2 = q_mekf
                q_mekf = np.array([
                    w1*w2 - x1*x2 - y1*y2 - z1*z2,
                    w1*x2 + x1*w2 + y1*z2 - z1*y2,
                    w1*y2 - x1*z2 + y1*w2 + z1*x2,
                    w1*z2 + x1*y2 - y1*x2 + z1*w2
                ])
                q_mekf = q_mekf / np.linalg.norm(q_mekf)

            bias_est_mekf += delta_bias

        # Update shared filter state
        with filter_state.lock:
            filter_state.q_mekf = q_mekf.copy()
            filter_state.bias_est_mekf = bias_est_mekf.copy()
            filter_state.current_time = current_time
            filter_state.triad_active = triad_available

        # Save to CSV: time, qw, qx, qy, qz
        csv_writer.writerow([current_time, q_mekf[0], q_mekf[1], q_mekf[2], q_mekf[3]])
        csv_file.flush()

        current_time += dt

        # Print status every second
        if int(current_time * 10) % 1 == 0:
            print(f"Time: {current_time:.1f}s | q_mekf: [{q_mekf[0]:.4f}, {q_mekf[1]:.4f}, {q_mekf[2]:.4f}, {q_mekf[3]:.4f}] | "
                  f"Bias: [{np.degrees(bias_est_mekf[0]):.3f}, {np.degrees(bias_est_mekf[1]):.3f}, {np.degrees(bias_est_mekf[2]):.3f}] deg/s | "
                  f"TRIAD: {'✓' if triad_available else '✗'}")

        time.sleep(dt)

################### Main Program ###################
print("Starting live attitude estimation... Press Ctrl+C to stop")

# Initialize IMU
MPU_Init()

# Open CSV file for writing
csv_file = open('quaternion_mekf.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['time', 'qw', 'qx', 'qy', 'qz'])

# Start worker threads
imu_worker = threading.Thread(target=imu_thread, daemon=True)
triad_worker = threading.Thread(target=triad_thread, daemon=True)
filter_worker = threading.Thread(target=filter_thread, args=(csv_writer, csv_file), daemon=True)

imu_worker.start()
triad_worker.start()
filter_worker.start()

# Give threads time to start
time.sleep(0.5)

try:
    # Keep main thread alive
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping attitude estimation...")

finally:
    running.clear()
    imu_worker.join(timeout=1)
    triad_worker.join(timeout=1)
    filter_worker.join(timeout=1)
    
    csv_file.close()
    
    with filter_state.lock:
        final_time = filter_state.current_time
    
    print(f"Total runtime: {final_time:.1f} seconds")
    print(f"Data saved to quaternion_mekf.csv")