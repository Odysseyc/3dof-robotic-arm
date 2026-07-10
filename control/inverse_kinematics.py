import sys
import os
sys.path.append(
 os.path.dirname(
  os.path.dirname(__file__)
 )
)

import numpy as np
from control.config_loader import load_robot_config

def inverse_kinematics(x, y, phi=0.0):
    """
    Calculates the true joint angles (theta1, theta2, theta3) for a 3-DOF planar arm.
    
    Parameters:
    x, y (float): Target coordinates for the tip of the end-effector.
    phi (float): Target orientation angle of the third link relative to the 
                 horizon (in radians). Default is 0.0 (horizontal).
    """
    # Load link lengths dynamically from config
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']
    L2 = config['robot']['link_lengths']['L2']
    L3 = config['robot']['link_lengths']['L3']
    
    # Step 1: Find the actual wrist joint (Joint 2) position using vector subtraction.
    # We step backward from the target tip along the orientation angle phi.
    x_wrist = x - L3 * np.cos(phi)
    y_wrist = y - L3 * np.sin(phi)
    
    # Distance from base origin to the wrist joint
    D_squared = x_wrist**2 + y_wrist**2
    D = np.sqrt(D_squared)
    
    # Verify the wrist is physically reachable by L1 and L2
    if D > (L1 + L2) or D < abs(L1 - L2):
        raise ValueError(f"Target position ({x}, {y}) with orientation {np.degrees(phi)}° is outside the workspace.")
        
    # Step 2: Solve for the elbow joint (theta2) using Law of Cosines
    cos_theta2 = (D_squared - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    
    # Standard Elbow-Down configuration choice
    theta2 = -np.arccos(cos_theta2)
    
    # Step 3: Solve for the shoulder joint (theta1)
    alpha = np.arctan2(y_wrist, x_wrist)
    beta = np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    theta1 = alpha - beta
    
    # Step 4: Solve for the wrist joint (theta3)
    # The total global angle is the sum of all relative angles: phi = theta1 + theta2 + theta3
    theta3 = phi - (theta1 + theta2)
    
    return theta1, theta2, theta3
