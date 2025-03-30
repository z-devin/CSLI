import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation
import numpy as np
from cube_visual import *
import time
import pyIGRF
from datetime import datetime
from astropy.time import Time
from astropy.coordinates import EarthLocation, AltAz, ITRS, TEME
from astropy.coordinates.matrix_utilities import matrix_transpose
from photodiode import get_light_vector

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

def compute_ecef_to_eci_matrix(timestamp):
    t = Time(timestamp, format="unix")
    gmst = t.sidereal_time('mean', 'greenwich').radian

    R = np.array([
        [ np.cos(gmst), np.sin(gmst), 0],
        [-np.sin(gmst), np.cos(gmst), 0],
        [0, 0, 1]
    ])
    return R

class Context:
    def __init__(self):
        self.q_current = np.array([0.0, 0.0, 0.0, 1.0])
        self.start_time = time.time()
        self.lat = 40.728447     # coopers coords
        self.lon = -73.990679
        self.alt = 500

def combined_update(frame, context, r1_inertial, r2_inertial, visualizer):
    # fake rotation for now
    current_time = time.time()
    elapsed = current_time - context.start_time
    angle = 0.02 * elapsed
    R_z = Rotation.from_euler('z', angle, degrees=False).as_matrix()

    b1_body = get_light_vector()
    b2_body = R_z @ r2_inertial

    R_BI = triad(b1_body, b2_body, r1_inertial, r2_inertial)

    # convert R_BI to quat
    R_IB = R_BI.T
    q_new = Rotation.from_matrix(R_IB).as_quat()  # [x, y, z, w]

    context.q_current = q_new
    visualizer.update(frame)

if __name__ == "__main__":
    currentyear = time.gmtime().tm_year
    context = Context()
    visual = CubeVisualizer(context)

    B_ecef = pyIGRF.igrf_value(context.lat, context.lon, context.alt, currentyear)[4:7]
    R_ecef_to_eci = compute_ecef_to_eci_matrix(context.start_time)
    B_eci = R_ecef_to_eci @ B_ecef

    # Inertial Vectors
    # r1_inertial = Solar Ephemeris
    r1_inertial = np.array([1.0, 0.0, 0.0])  # Sun/flashlight
    r2_inertial = B_eci / np.linalg.norm(B_eci)  # Mag field using geomagnetic model

    animation = FuncAnimation(
        visual.fig,
        combined_update,
        fargs=(context, r1_inertial, r2_inertial, visual),
        interval=1,
        cache_frame_data=False
    )

    plt.show()
