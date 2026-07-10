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
    slider_x = p.addUserDebugParameter("Target X", -max_reach, max_reach, 0.20)
    slider_y = p.addUserDebugParameter("Target Y", -max_reach, max_reach, 0.15)
    slider_phi = p.addUserDebugParameter("Target Phi (Rad)", -np.pi, np.pi, 0.0)

    # --- NEW: Variable to hold the previous frame's tip position ---
    # In our URDF, link3 is index 2.
    end_effector_link_idx = 2
    last_tip_position = None

    print("\n=======================================================")
    print("Interactive Mode Active! Move sliders in the PyBullet window.")
    print("=======================================================\n")

    try:
        while p.isConnected():
            target_x = p.readUserDebugParameter(slider_x)
            target_y = p.readUserDebugParameter(slider_y)
            target_phi = p.readUserDebugParameter(slider_phi)
            
            try:
                theta1, theta2, theta3 = inverse_kinematics(target_x, target_y, phi=target_phi)
                target_angles = [theta1, theta2, theta3]
                
                for i, joint_idx in enumerate(active_joints):
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angles[i],
                        force=150.0
                    )
            except ValueError:
                pass

            # --- NEW: Trace the End-Effector Path ---
            # 1. Get the current spatial coordinates of the tip link
            link_state = p.getLinkState(robot_id, end_effector_link_idx)
            current_tip_position = link_state[0]  # Extracts (X, Y, Z) tuple

            # 2. If we have a past frame coordinate, draw a segment between them
            if last_tip_position is not None:
                p.addUserDebugLine(
                    lineFromXYZ=last_tip_position,
                    lineToXYZ=current_tip_position,
                    lineColorRGB=[1, 0, 0],  # Bright Red Line
                    lineWidth=2.5,
                    lifeTime=10.0            # Line fades out automatically after 10 seconds
                )
            
            # 3. Update the tracking coordinate for the next loop cycle
            last_tip_position = current_tip_position

            p.stepSimulation()
            time.sleep(1.0 / 240.0)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    run_pybullet_simulation()
