# triad_sensor.py
import numpy as np
from scipy.spatial.transform import Rotation
import time
from photodiode_6 import get_light_vector
from magnetometer_visual import read_magnetometer, magnetometer_setup
from smbus2 import SMBus

def normalize(vec):
    return vec / np.linalg.norm(vec)

def triad(b1, b2, r1, r2):
    # Inputs:
    # b1, b2 - body frame vectors
    # r1, r2 - inertial frame vectors
    # Outputs:
    # R_BI - 3x3 rot matrix from body to inertial

    b1 = normalize(b1)
    b2 = normalize(b2)
    r1 = normalize(r1)
    r2 = normalize(r2)

    # body frame axis
    t1 = b1
    t2 = normalize(np.cross(b1, b2))
    t3 = np.cross(t1, t2)

    # inertial frame axis
    R1 = r1
    R2 = normalize(np.cross(r1, r2))
    R3 = np.cross(R1, R2)

    T_body = np.column_stack((t1, t2, t3))
    T_inertial = np.column_stack((R1, R2, R3))
    C_BI = T_body @ T_inertial.T
    return C_BI

# Initialize bus and reference vectors globally
bus = SMBus(1)
magnetometer_setup(bus)

# Reference vectors in inertial frame
r1_inertial = np.array([0.0, -1.0, 0.0])
B_NED = np.array([-0.70553762, 0.10511136, 0.70082871])
r2_inertial = B_NED / np.linalg.norm(B_NED)

def get_triad_data():
    """
    Compute attitude quaternion using TRIAD algorithm
    
    Returns:
        triad_available (bool): True if computation successful, False otherwise
        q_triad_meas (np.array): Quaternion [w, x, y, z] if successful, else zeros
    """
    # Get body frame measurements
    b1_body, _, _ = get_light_vector()
    b2_body = read_magnetometer(bus)
    
    # Compute rotation matrix using TRIAD
    R_BI = triad(b1_body, b2_body, r1_inertial, r2_inertial)
    
    # Convert to quaternion (from body to inertial)
    R_IB = R_BI.T
    q_scipy = Rotation.from_matrix(R_IB).as_quat()  # Returns [x, y, z, w]
    
    # Convert to [w, x, y, z] format
    q_triad_meas = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])
    
    return True, q_triad_meas

# Only run this block if the file is executed directly (not imported)
if __name__ == "__main__":
    import csv
    
    with open('quaternion_data_still.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['index', 'sx', 'sy', 'sz', 'mx', 'my', 'mz', 'qx', 'qy', 'qz', 'qw'])
        
        index = 0
        
        while True:
            triad_available, q_triad_meas = get_triad_data()
            
            if triad_available:
                # Get the raw measurements for logging
                b1_body, _, _ = get_light_vector()
                b2_body = read_magnetometer(bus)
                
                # Convert quaternion back to [x, y, z, w] for logging
                q_xyzw = [q_triad_meas[1], q_triad_meas[2], q_triad_meas[3], q_triad_meas[0]]
                
                writer.writerow([index] + list(b1_body) + list(b2_body) + q_xyzw)
                f.flush()
                print(f"Index: {index} - TRIAD successful")
            else:
                print(f"Index: {index} - TRIAD failed")
            
            index += 1