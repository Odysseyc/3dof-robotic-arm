import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Add project root to Python path
sys.path.append(
    os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )
)

from control.config_loader import load_robot_config
from control.forward_kinematics import forward_kinematics
from control.inverse_kinematics import inverse_kinematics
from control.trajectory import interpolate_joints

# Load link lengths dynamically from configuration
config = load_robot_config()
L1 = config['robot']['link_lengths']['L1']
L2 = config['robot']['link_lengths']['L2']
L3 = config['robot']['link_lengths']['L3']

def get_points(angles):
    theta1, theta2, theta3 = angles

    x0, y0 = 0.0, 0.0

    # Link 1 position (Elbow joint)
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    # Link 2 position (Wrist joint) using cumulative frame tracking
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    # Link 3 position (End-effector tip)
    x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + L3 * np.sin(theta1 + theta2 + theta3)

    return [x0, x1, x2, x3], [y0, y1, y2, y3]


# Starting configurations (joint space)
start = np.array([0.0, 0.0, 0.0])

# Target Cartesian goal & desired orientation angle relative to the horizon
target_position = (0.22, 0.10)
target_phi = 0.0  # 0.0 keeps the end-effector perfectly horizontal at destination

# Resolve coordinates to joint positions using true 3-DOF analytical solver
goal = np.array(
    inverse_kinematics(target_position[0], target_position[1], phi=target_phi)
)

# Plan linear joint spaces trajectory interpolation profile
trajectory = interpolate_joints(
    start,
    goal,
    steps=100
)

# Plot Canvas Adjustments
fig, ax = plt.subplots()

# Set plot window dynamically based on global workspace profile max length bounds
max_reach = L1 + L2 + L3
ax.set_xlim(-max_reach * 1.1, max_reach * 1.1)
ax.set_ylim(-max_reach * 1.1, max_reach * 1.1)
ax.set_aspect("equal")
ax.grid(True, linestyle="--", alpha=0.5)
ax.set_title("3-DOF Kinematics Profile Visualizer")

line, = ax.plot([], [], "-o", linewidth=3, markersize=6, color="#2ca02c")


def update(frame):
    x, y = get_points(
        trajectory[frame]
    )
    line.set_data(x, y)
    return line,


animation = FuncAnimation(
    fig,
    update,
    frames=len(trajectory),
    interval=50,
    blit=True
)

# Save generation out file cleanly utilizing pillow
output_path = "docs/images/arm_animation.gif"
os.makedirs(os.path.dirname(output_path), exist_ok=True)
animation.save(
    output_path,
    writer="pillow",
    fps=20
)

plt.show()
