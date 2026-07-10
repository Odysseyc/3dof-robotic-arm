import sys
import os
sys.path.append(
 os.path.dirname(
  os.path.dirname(__file__)
 )
)

import numpy as np
from control.config_loader import load_robot_config

def inverse_kinematics(target_x, target_y, target_z, phi=0.0):
    """
    Calculates the analytical inverse kinematics for a 3-DOF Anthropomorphic Arm.
    
    Parameters:
        target_x (float): Target X coordinate in meters
        target_y (float): Target Y coordinate in meters
        target_z (float): Target Z coordinate in meters (Height)
        phi (float): Desired pitch angle of the end-effector link (link3) 
                     relative to the horizontal plane (in radians).
                     
    Returns:
        tuple: (theta1, theta2, theta3) joint angles in radians.
        
    Raises:
        ValueError: If the target coordinate is outside the arm's reachable workspace.
    """
    # 1. Load system config parameters
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']  # Vertical pedestal stand height
    L2 = config['robot']['link_lengths']['L2']  # Upper arm length
    L3 = config['robot']['link_lengths']['L3']  # Forearm/wrist length

    # 2. Solve for Joint 1 (Base Waist/Yaw Angle)
    # This aligns the vertical plane of the robot with the 3D target point
    theta1 = np.arctan2(target_y, target_x)

    # 3. Project the 3D target into a 2D Vertical Plane (R, Z)
    # R is the combined horizontal distance from the base center to the target
    R = np.sqrt(target_x**2 + target_y**2)
    
    # Z_rel is the height relative to the top of our vertical pedestal (Joint 2 origin)
    Z_rel = target_z - L1

    # 4. Subtract the End-Effector Link 3 Vector
    # Since Joint 3 pitches relative to the Y-axis, phi controls its approach angle 
    # relative to the horizontal floor plane.
    wrist_r = R - L3 * np.cos(phi)
    wrist_z = Z_rel - L3 * np.sin(phi)

    # 5. Solve the remaining 2-Link Planar Sub-Problem using Cosine Law
    # Distance from the shoulder joint (Joint 2) to the wrist joint (Joint 3)
    D_squared = wrist_r**2 + wrist_z**2
    D = np.sqrt(D_squared)

    # Triangle inequality workspace boundary check
    if D > (L2 + L3) or D < abs(L2 - L3):
        raise ValueError("Target point is outside the physical workspace envelope.")

    # Cosine Law to find the inner elbow angle
    cos_alpha = (L2**2 + L3**2 - D_squared) / (2.0 * L2 * L3)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)  # Handle minor floating point rounding errors
    alpha = np.arccos(cos_alpha)
    
    # theta3 is the relative interior/exterior bending angle of the elbow joint
    theta3 = alpha - np.pi

    # Solve for Joint 2 (Shoulder/Pitch Angle)
    # gamma is the angle up to the wrist point; beta is the internal correction offset
    gamma = np.arctan2(wrist_z, wrist_r)
    
    cos_beta = (L2**2 + D_squared - L3**2) / (2.0 * L2 * D)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)
    
    # theta2 is the pitching angle relative to our vertical arm stand structure
    theta2 = gamma + beta

    return float(theta1), float(theta2), float(theta3)
