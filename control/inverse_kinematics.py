import sys
import os
sys.path.append(
 os.path.dirname(
  os.path.dirname(__file__)
 )
)

import numpy as np
from control.config_loader import load_robot_config

def inverse_kinematics(x, y):
    """
    Calculates the required joint angles (theta1, theta2, theta3) to reach 
    a target (x, y) position. 
    Loads link lengths dynamically from the robot configuration.
    
    Note: Keeps the 3rd link constrained to horizontal to reduce the 
    underdetermined 3-DOF problem to a geometric 2-DOF solver.
    """
    # Load link lengths dynamically from config
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']
    L2 = config['robot']['link_lengths']['L2']
    L3 = config['robot']['link_lengths']['L3']
    
    # Step 1: Calculate effective wrist position (assuming L3 remains horizontal)
    x_wrist = x - L3
    y_wrist = y
    
    # Distance from base origin to the wrist joint
    D_squared = x_wrist**2 + y_wrist**2
    D = np.sqrt(D_squared)
    
    # Check if the target point is physically reachable by L1 and L2
    if D > (L1 + L2) or D < abs(L1 - L2):
        raise ValueError(f"Target position ({x}, {y}) is outside the reachable workspace.")
        
    # Step 2: Solve the geometric 2-DOF problem for the elbow joint (theta2)
    # Using the Law of Cosines: D^2 = L1^2 + L2^2 - 2*L1*L2*cos(180 - theta2)
    cos_theta2 = (D_squared - L1**2 - L2**2) / (2 * L1 * L2)
    # Clip values slightly to avoid floating-point errors outside [-1, 1]
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    
    # Elbow-down configuration choice
    theta2 = -np.arccos(cos_theta2)
    
    # Step 3: Solve for the shoulder joint (theta1)
    alpha = np.arctan2(y_wrist, x_wrist)
    beta = np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    theta1 = alpha - beta
    
    # Step 4: Constrain the 3rd link to face perfectly forward (horizontal)
    # Total angle relative to base frame must equal 0: theta1 + theta2 + theta3 = 0
    theta3 = -(theta1 + theta2)
    
    return theta1, theta2, theta3
