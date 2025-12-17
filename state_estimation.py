import numpy as np
import matplotlib.pyplot as plt
from imu import get_imu_data, MPU_Init
from triad_function import get_triad_data
import threading
import time

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
        
sensor_data = SensorData()
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
        time.sleep(dt)  # Match IMU sampling rate

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
        time.sleep(0.1)  # TRIAD can run slower (10 Hz)

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

################### Initialize State ###################
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

# Data history
history_length = 10000
time_history = []
omega_history = []
bias_comp_history = []
bias_mekf_history = []
q_comp_history = []
q_mekf_history = []
q_imu_history = []

current_time = 0.0

print("Starting live attitude estimation... Press Ctrl+C to stop")

# Initialize IMU
MPU_Init()

# Start sensor threads
imu_worker = threading.Thread(target=imu_thread, daemon=True)
triad_worker = threading.Thread(target=triad_thread, daemon=True)

imu_worker.start()
triad_worker.start()

# Give threads time to start
time.sleep(0.5)

################### Main Filter Loop ###################
try:
    while True:
        # Get latest sensor data (non-blocking)
        with sensor_data.lock:
            omega_measured = sensor_data.omega.copy()
            triad_available = sensor_data.new_triad_data
            q_triad_meas = sensor_data.q_triad.copy()
            sensor_data.new_triad_data = False  # Reset flag
        
        ################### COMPLEMENTARY FILTER ###################
        correction_rate = np.zeros(3)
        
        if triad_available:
            # Compute attitude error
            q_comp_conj = np.array([q_comp[0], -q_comp[1], -q_comp[2], -q_comp[3]])
            w1, x1, y1, z1 = q_triad_meas
            w2, x2, y2, z2 = q_comp_conj
            q_err = np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            ])
            
            if q_err[0] < 0:
                q_err = -q_err
            
            error_vec = 2 * q_err[1:]
            correction_rate = (1 - alpha) / dt * error_vec
        
        omega_corrected = omega_measured + correction_rate
        q_comp = integrate_quaternion(q_comp, omega_corrected, dt)
        
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
        
        ################### Store history ###################
        time_history.append(current_time)
        omega_history.append(omega_measured.copy())
        bias_comp_history.append(bias_est_comp.copy())
        bias_mekf_history.append(bias_est_mekf.copy())
        q_comp_history.append(q_comp.copy())
        q_mekf_history.append(q_mekf.copy())
        q_imu_history.append(q_imu.copy())
        
        if len(time_history) > history_length:
            time_history.pop(0)
            omega_history.pop(0)
            bias_comp_history.pop(0)
            bias_mekf_history.pop(0)
            q_comp_history.pop(0)
            q_mekf_history.pop(0)
            q_imu_history.pop(0)
        
        current_time += dt
        
        if int(current_time * 10) % 10 == 0:
            print(f"Time: {current_time:.1f}s | MEKF bias: [{np.degrees(bias_est_mekf[0]):.3f}, "
                  f"{np.degrees(bias_est_mekf[1]):.3f}, {np.degrees(bias_est_mekf[2]):.3f}] deg/s | "
                  f"TRIAD: {'✓' if triad_available else '✗'}")
        

except KeyboardInterrupt:
    print("\nStopping attitude estimation...")
    running.clear()
    imu_worker.join(timeout=1)
    triad_worker.join(timeout=1)
    print(f"Total runtime: {current_time:.1f} seconds")

################### Plotting ###################
time = np.array(time_history)
omega_measured = np.array(omega_history)
bias_est_comp = np.array(bias_comp_history)
bias_est_mekf = np.array(bias_mekf_history)

fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
axes = ['x', 'y', 'z']
for i in range(3):
    ax[i].plot(time, np.degrees(omega_measured[:, i]), label='Measured', alpha=0.7)
    ax[i].set_ylabel(f'omega {axes[i]} [deg/s]')
    ax[i].grid(True)
    ax[i].legend()
    ax[i].set_title(f'Angular Rate ({axes[i].upper()}-axis)')

ax[-1].set_xlabel('Time [s]')
plt.tight_layout()
plt.show()

fig2, ax2 = plt.subplots(figsize=(12, 8))
ax2.plot(time, np.degrees(bias_est_comp), label=['Comp X', 'Comp Y', 'Comp Z'], linewidth=1)
ax2.plot(time, np.degrees(bias_est_mekf), label=['MEKF X', 'MEKF Y', 'MEKF Z'], linewidth=2)
ax2.set_ylabel('Gyro Bias [deg/s]')
ax2.set_xlabel('Time [s]')
ax2.legend()
ax2.grid(True)
ax2.set_title('Gyro Bias Estimation')
plt.tight_layout()
plt.show()