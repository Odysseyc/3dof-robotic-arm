import sys
import os
import time
import numpy as np
import pybullet as p
import pybullet_data

# Ensure project root is in the path to read your custom IK module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.inverse_kinematics import inverse_kinematics
from control.config_loader import load_robot_config

def run_pybullet_simulation():
    # 1. Initialize PyBullet Physics Client
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Configure workspace viewport camera
    p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=0, cameraPitch=-89, cameraTargetPosition=[0.1, 0, 0.1])
    
    # Load environment plane
    p.loadURDF("plane.urdf")
    
    # 2. Load Your Custom 3-DOF Robot Arm
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_path = os.path.join(project_root, "urdf", "arm_3dof.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    
    active_joints = [0, 1, 2] # Joint indices from your URDF
    
    # Load config to get maximum reach boundaries
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']
    L2 = config['robot']['link_lengths']['L2']
    L3 = config['robot']['link_lengths']['L3']
    max_reach = L1 + L2 + L3

    # 3. Initialize PyBullet Interactive GUI Sliders
    # Parameters: p.addUserDebugParameter("Label", minimum, maximum, initial_value)
    slider_x = p.addUserDebugParameter("Target X", -max_reach, max_reach, 0.20)
    slider_y = p.addUserDebugParameter("Target Y", -max_reach, max_reach, 0.15)
    slider_phi = p.addUserDebugParameter("Target Phi (Rad)", -np.pi, np.pi, 0.0)

    print("\n=======================================================")
    print("Interactive Mode Active! Move sliders in the PyBullet window.")
    print("Press Ctrl+C in the terminal to terminate.")
    print("=======================================================\n")

    # 4. Infinite Real-Time Control Loop
    try:
        while p.isConnected():
            # Read current values directly from the user GUI panels
            target_x = p.readUserDebugParameter(slider_x)
            target_y = p.readUserDebugParameter(slider_y)
            target_phi = p.readUserDebugParameter(slider_phi)
            
            try:
                # Resolve joint angles dynamically using your analytic IK function
                theta1, theta2, theta3 = inverse_kinematics(target_x, target_y, phi=target_phi)
                target_angles = [theta1, theta2, theta3]
                
                # Command joint motors to move to the computed IK angles
                for i, joint_idx in enumerate(active_joints):
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angles[i],
                        force=150.0 # Enough torque to hold posture against gravity
                    )
            except ValueError:
                # If target coordinates move out of the arm's reach profile,
                # catch the error quietly so the simulation stays alive without crashing.
                pass

            p.stepSimulation()
            time.sleep(1.0 / 240.0) # Standard 240Hz update tick rate
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    run_pybullet_simulation()
