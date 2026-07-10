import sys 
import os 

sys.path.append(
  os.path.dirname(
    os.path.dirname(__file__)
  )
)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from control.config_loader import load_robot_config

def generate_motion_gif(angle_history, target_path=None, filename="arm_motion.gif"):
    """
    Generates and saves a GIF animation of the 3-DOF robotic arm tracking a path.
    Calculates exact joint positions using cumulative trigonometry.
    """
    # Load link lengths dynamically from configuration
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']
    L2 = config['robot']['link_lengths']['L2']
    L3 = config['robot']['link_lengths']['L3']
    
    fig, ax = plt.subplots(figsize=(6, 6))
    
    # Set up plot limits with a buffer based on total arm reach
    max_reach = L1 + L2 + L3
    ax.set_xlim(-max_reach * 1.2, max_reach * 1.2)
    ax.set_ylim(-max_reach * 1.2, max_reach * 1.2)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_title("Cartesian Path Tracking Control Loop")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    
    # Plot elements
    line, = ax.plot([], [], 'o-', lw=3, color='#1f77b4', markersize=8, label="Robot Arm")
    effector_dot, = ax.plot([], [], 'ro', markersize=6, label="End Effector Track")
    
    if target_path is not None:
        target_path = np.array(target_path)
        ax.plot(target_path[:, 0], target_path[:, 1], 'k--', alpha=0.5, label="Target Path")
    
    ax.legend(loc="upper right")
    
    # History tracking for the end-effector's actual trailing path
    x_tail, y_tail = [], []

    def init():
        line.set_data([], [])
        effector_dot.set_data([], [])
        return line, effector_dot

    def update(frame):
        # Extract angles for the current time step
        theta1, theta2, theta3 = angle_history[frame]
        
        # Calculate cumulative angles relative to the global base frame
        alpha1 = theta1
        alpha2 = theta1 + theta2
        alpha3 = theta1 + theta2 + theta3
        
        # Base origin (Joint 0)
        x0, y0 = 0.0, 0.0
        
        # Elbow joint (Joint 1) position
        x1 = L1 * np.cos(alpha1)
        y1 = L1 * np.sin(alpha1)
        
        # Wrist joint (Joint 2) position
        x2 = x1 + L2 * np.cos(alpha2)
        y2 = y1 + L2 * np.sin(alpha2)
        
        # End-effector (Joint 3) position
        x3 = x2 + L3 * np.cos(alpha3)
        y3 = y2 + L3 * np.sin(alpha3)
        
        # Update the structural lines of the robot arm
        line.set_data([x0, x1, x2, x3], [y0, y1, y2, y3])
        
        # Update trailing path of the end-effector
        x_tail.append(x3)
        y_tail.append(y3)
        effector_dot.set_data(x_tail, y_tail)
        
        return line, effector_dot

    # Construct the animation loop
    num_frames = len(angle_history)
    ani = FuncAnimation(fig, update, frames=num_frames, init_func=init, blit=True, interval=40)
    
    # Save output utilizing pillow writer
    ani.save(filename, writer='pillow', fps=25)
    plt.close(fig)
    print(f"Success: Animation successfully rendered and saved to {filename}")
