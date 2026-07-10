import sys
import os
sys.path.append(
 os.path.dirname(
  os.path.dirname(__file__)
 )
)

import numpy as np
from control.config_loader import load_robot_config

def forward_kinematics(theta1, theta2, theta3):
    """
    Calculates the end-effector (x, y) coordinates given the joint angles.
    Loads link lengths dynamically from the robot configuration.
    
    Parameters:
    theta1, theta2, theta3 (float): Joint angles in radians.
    
    Returns:
    tuple: (x, y) coordinates of the end-effector.
    """
    # Load link lengths dynamically from config
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']
    L2 = config['robot']['link_lengths']['L2']
    L3 = config['robot']['link_lengths']['L3']
    
    # Cumulative angles for each link relative to the base frame
    alpha1 = theta1
    alpha2 = theta1 + theta2
    alpha3 = theta1 + theta2 + theta3
    
    # Calculate positions of each link endpoint using trigonometry
    x = L1 * np.cos(alpha1) + L2 * np.cos(alpha2) + L3 * np.cos(alpha3)
    y = L1 * np.sin(alpha1) + L2 * np.sin(alpha2) + L3 * np.sin(alpha3)
    
    return x, y
