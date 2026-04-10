import numpy as np
import pandas as pd

################### Load Data ###################
imu_data = pd.read_csv('z_imu_log.csv')
triad_data = pd.read_csv('z_triad_log.csv')

# Build sorted event list
events = []
for _, row in imu_data.iterrows():
    events.append({
        'time': row['timestamp'],
        'type': 'IMU',
        'data': np.array([row['gx'], row['gy'], row['gz']])
    })
for _, row in triad_data.iterrows():
    events.append({
        'time': row['timestamp'],
        'type': 'TRIAD',
        'data': np.array([row['qw'], row['qx'], row['qy'], row['qz']])
    })
events.sort(key=lambda e: e['time'])

################### Filter Config ###################
gyro_noise_cov = np.diag([0.006906**2, 0.013501**2, 0.004708**2])
bias_drift_cov = np.diag([1.501764e-07**2, 6.737916e-08**2, 2.869799e-09**2])
R = np.diag([1.8190332587e-06, 1.5939245208e-05, 1.9316665858e-08])

Q = np.block([
    [gyro_noise_cov, np.zeros((3, 3))],
    [np.zeros((3, 3)), bias_drift_cov]
])

P = np.eye(6)
P[0:3, 0:3] *= np.radians(10.0)**2
P[3:6, 3:6] *= np.radians(0.5)**2

################### Quaternion Math ###################
def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def integrate_quaternion(q, omega, dt):
    omega_norm = np.linalg.norm(omega)
    if omega_norm < 1e-10:
        return q / np.linalg.norm(q)
    half_angle = 0.5 * omega_norm * dt
    s = np.sin(half_angle) / omega_norm
    q_delta = np.array([np.cos(half_angle), s*omega[0], s*omega[1], s*omega[2]])
    q_new = quat_multiply(q, q_delta)
    return q_new / np.linalg.norm(q_new)

def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

################### Run MEKF ###################
q_mekf = np.array([1.0, 0.0, 0.0, 0.0])
bias_est = np.zeros(3)
last_time = events[0]['time']

results = []

for i, event in enumerate(events):
    dt = event['time'] - last_time
    last_time = event['time']

    if dt < 0:
        raise ValueError(f"Negative dt={dt} at event {i}, time={event['time']}")
    if dt == 0:
        continue

    if event['type'] == 'IMU':
        omega = event['data']
        omega_corrected = omega - bias_est

        # Predict
        q_mekf = integrate_quaternion(q_mekf, omega_corrected, dt)

        F = np.zeros((6, 6))
        F[0:3, 0:3] = np.eye(3) - skew(omega_corrected) * dt
        F[0:3, 3:6] = -dt * np.eye(3)
        F[3:6, 3:6] = np.eye(3)
        P = F @ P @ F.T + Q

    elif event['type'] == 'TRIAD':
        q_meas = event['data']

        # Error quaternion
        q_ref_conj = np.array([q_mekf[0], -q_mekf[1], -q_mekf[2], -q_mekf[3]])
        delta_q = quat_multiply(q_meas, q_ref_conj)

        if delta_q[0] < 0:
            delta_q = -delta_q

        z = 2 * delta_q[1:]

        # Update
        H = np.zeros((3, 6))
        H[0:3, 0:3] = np.eye(3)
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        delta_x = K @ z
        P = (np.eye(6) - K @ H) @ P

        # Apply attitude correction
        delta_theta = delta_x[0:3]
        delta_theta_norm = np.linalg.norm(delta_theta)
        if delta_theta_norm > 1e-10:
            delta_q_update = np.array([
                np.cos(delta_theta_norm / 2),
                *(delta_theta / delta_theta_norm * np.sin(delta_theta_norm / 2))
            ])
            q_mekf = quat_multiply(delta_q_update, q_mekf)
            q_mekf /= np.linalg.norm(q_mekf)

        # Apply bias correction
        bias_est += delta_x[3:6]

    results.append([event['time'], event['type'],
                    q_mekf[0], q_mekf[1], q_mekf[2], q_mekf[3],
                    bias_est[0], bias_est[1], bias_est[2]])

    # if i % 1000 == 0:
    print(f"Event {i}/{len(events)} | q: [{q_mekf[0]:.4f}, {q_mekf[1]:.4f}, {q_mekf[2]:.4f}, {q_mekf[3]:.4f}] | "
            f"bias: [{np.degrees(bias_est[0]):.3f}, {np.degrees(bias_est[1]):.3f}, {np.degrees(bias_est[2]):.3f}] deg/s")

################### Save Results ###################
results_df = pd.DataFrame(results,
    columns=['time', 'event_type', 'qw', 'qx', 'qy', 'qz', 'bx', 'by', 'bz'])
results_df.to_csv('offline_mekf_results.csv', index=False)

print(f"\nProcessed {len(events)} events")
print(f"IMU events: {sum(1 for e in events if e['type'] == 'IMU')}")
print(f"TRIAD events: {sum(1 for e in events if e['type'] == 'TRIAD')}")
print(f"Final bias: [{np.degrees(bias_est[0]):.4f}, {np.degrees(bias_est[1]):.4f}, {np.degrees(bias_est[2]):.4f}] deg/s")
print("Saved to offline_mekf_results.csv")