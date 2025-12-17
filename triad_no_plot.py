# import RPi.GPIO as GPIO
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
from photodiode_6 import get_light_vector
from magnetometer_visual import read_magnetometer, magnetometer_setup
from smbus2 import SMBus
import csv

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

if __name__ == "__main__":
    currentyear = time.gmtime().tm_year
    context = Context()
    
    with SMBus(1) as bus:
        magnetometer_setup(bus)
        
        B_NED = np.array([-0.70553762, 0.10511136, 0.70082871])
        
        r1_inertial = np.array([0.0, -1.0, 0.0])
        r2_inertial = B_NED/np.linalg.norm(B_NED)
        
        with open('quaternion_data_still.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 'sx', 'sy', 'sz', 'mx', 'my', 'mz', 'qx', 'qy', 'qz', 'qw'])
            
            index = 0
            
            while True:
                b1_body, _, _ = get_light_vector()
                b2_body = read_magnetometer(bus)
                
                R_BI = triad(b1_body, b2_body, r1_inertial, r2_inertial)
                
                R_IB = R_BI.T
                try:
                    q_new = Rotation.from_matrix(R_IB).as_quat()
                    context.q_current = q_new
                except:
                    pass
                
                writer.writerow([index] + list(b1_body) + list(b2_body) + list(context.q_current))
                f.flush()
                print(f"Index: {index}")
                index += 1