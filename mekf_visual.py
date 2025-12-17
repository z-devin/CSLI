import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from cube_visual import CubeVisualizer
import pandas as pd
import time

class Context:
    def __init__(self):
        self.q_current = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w] format for scipy
        self.start_time = time.time()

################### Playback Configuration ###################
PLAYBACK_SPEED = 10  # 1.0 = real-time, 2.0 = 2x speed, 0.5 = half speed
FRAME_SKIP = 1        # Skip frames: 1 = show all, 2 = show every other frame, etc.
FPS = 60              # Target FPS for animation (max ~60-100 for smooth matplotlib)

# Load quaternion data from CSV
print("Loading quaternion data...")
df = pd.read_csv('quaternion_mekf.csv')
print(f"Loaded {len(df)} frames")

# Extract data
times = df['time'].values
qw = df['qw'].values
qx = df['qx'].values
qy = df['qy'].values
qz = df['qz'].values

# Calculate interval based on desired FPS and playback speed
# Original data rate is 100 Hz (10ms between frames)
original_dt = times[1] - times[0] if len(times) > 1 else 0.01
interval_ms = (1000 / FPS)  # Milliseconds per frame for display

print(f"Playback speed: {PLAYBACK_SPEED}x")
print(f"Frame skip: {FRAME_SKIP} (showing every {FRAME_SKIP} frame(s))")
print(f"Target FPS: {FPS}")
print(f"Data duration: {times[-1]:.1f}s, Playback duration: {times[-1]/PLAYBACK_SPEED:.1f}s")

# Initialize visualization
context = Context()
visualizer = CubeVisualizer(context)

# Current frame index
current_frame = [0]
start_time = [time.time()]

def update_visualization(frame):
    # Calculate which data frame to show based on playback speed and frame skip
    frames_per_update = int(FRAME_SKIP * PLAYBACK_SPEED)
    idx = current_frame[0]
    
    if idx >= len(times):
        idx = 0  # Loop back to start
        current_frame[0] = 0
        start_time[0] = time.time()
    
    # Convert from [w, x, y, z] to [x, y, z, w] for scipy
    context.q_current = np.array([qx[idx], qy[idx], qz[idx], qw[idx]])
    
    # Update visualization
    visualizer.ax.cla()
    visualizer.update(frame)
    
    # Calculate actual playback time
    elapsed_real = time.time() - start_time[0]
    
    # Set title with current time and playback info
    visualizer.ax.set_title(
        f'MEKF Attitude - Data Time: {times[idx]:.2f}s | '
        f'Playback: {elapsed_real:.2f}s | Speed: {PLAYBACK_SPEED}x',
        pad=20, fontsize=10
    )
    
    current_frame[0] += frames_per_update

# Create animation with calculated interval
animation = FuncAnimation(
    visualizer.fig,
    update_visualization,
    interval=interval_ms,
    blit=False,
    cache_frame_data=False
)

print("Playing back quaternion data... Close window to exit.")
print("Adjust PLAYBACK_SPEED and FRAME_SKIP at top of file for different speeds.")
plt.show()

