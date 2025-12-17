from gpiozero import MCP3008
from gpiozero.pins.native import NativeFactory
import numpy as np
import csv

factory = NativeFactory()

DIRECTIONS = np.array([
    [0, -1, 0],
    [0, 0, -1],
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
])

def get_light_vector():
    diode0 = MCP3008(0, pin_factory=factory).value
    diode1 = MCP3008(1, pin_factory=factory).value
    diode2 = MCP3008(2, pin_factory=factory).value
    diode3 = MCP3008(3, pin_factory=factory).value
    diode4 = MCP3008(4, pin_factory=factory).value
    diode5 = MCP3008(5, pin_factory=factory).value

    intensities = np.array([diode0, diode1, diode2, diode3, diode4, diode5])

    idx_top3 = np.argsort(intensities)[-3:]

    v = np.zeros(3)
    for i in idx_top3:
        v += intensities[i] * DIRECTIONS[i]

    mag = np.linalg.norm(v) + 1e-12
    sun_vec = v / mag

    return sun_vec, intensities, mag

with open('photodiode_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['index', 'x', 'y', 'z'])
    
    index = 0
    while True:
        sun_vec, _, _ = get_light_vector()
        writer.writerow([index, sun_vec[0], sun_vec[1], sun_vec[2]])
        f.flush()
        print(index)
        index += 1

