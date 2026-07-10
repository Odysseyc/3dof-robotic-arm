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
    
    # Configure workspace viewport camera to look directly down at the 2D plane
    p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=0, cameraPitch=-89, cameraTargetPosition=[0.1, 0, 0.1])
    
    # Load environment floor plane
    p.loadURDF("plane.urdf")
    
    # 2. Load Your Custom 3-DOF Robot Arm
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_path = os.path.join(project_root, "urdf", "arm_3dof.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    
    active_joints = [0, 1, 2] # Joint indices tracking joint1, joint2, joint3
    
    # Load config parameters to evaluate physical system limits
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']
    L2 = config['robot']['link_lengths']['L2']
    L3 = config['robot']['link_lengths']['L3']
    max_reach = L1 + L2 + L3

    # 3. Initialize PyBullet Interactive GUI Sliders & Mode Controls
    # Mode toggle: 0 = Manual Sliders, 1 = Auto Circle Track
    slider_mode = p.addUserDebugParameter("AUTO MODE (0=Off, 1=On)", 0, 1, 0)
    
    slider_x = p.addUserDebugParameter("Target X (Manual)", -max_reach, max_reach, 0.20)
    slider_y = p.addUserDebugParameter("Target Y (Manual)", -max_reach, max_reach, 0.15)
    slider_phi = p.addUserDebugParameter("Target Phi (Rad)", -np.pi, np.pi, 0.0)

    # Variables to track structural paths frame-over-frame
    end_effector_link_idx = 2
    last_tip_position = None
    
    # Parametric variables to define the tracking circle
    circle_center_x = 0.18
    circle_center_y = 0.12
    circle_radius = 0.06
    sim_time = 0.0  # Tracks overall accumulated runtime

    print("\n=======================================================")
    print("Simulation Pipeline Running!")
    print("Toggle 'AUTO MODE' slider to 1 to watch the arm trace a circle.")
    print("=======================================================\n")

    # 4. Infinite Real-Time Control Loop
    try:
        while p.isConnected():
            # Check the current operational mode
            auto_mode_active = p.readUserDebugParameter(slider_mode) >= 0.5
            target_phi = p.readUserDebugParameter(slider_phi)
            
            if auto_mode_active:
                # --- AUTO-CIRCLE GENERATOR ---
                # Increment time smoothly based on physics loop frequency
                sim_time += 1.0 / 240.0
                speed_factor = 2.0  # Adjusts how fast the arm moves around the circle
                
                # Parametric equations of a circle: x = cx + r*cos(w*t), y = cy + r*sin(w*t)
                target_x = circle_center_x + circle_radius * np.cos(speed_factor * sim_time)
                target_y = circle_center_y + circle_radius * np.sin(speed_factor * sim_time)
            else:
                # --- MANUAL SLIDER MODE ---
                target_x = p.readUserDebugParameter(slider_x)
                target_y = p.readUserDebugParameter(slider_y)
                # Reset simulation time when switching back to manual mode
                sim_time = 0.0
            
            try:
                # Resolve joint angles using your analytical IK solver
                theta1, theta2, theta3 = inverse_kinematics(target_x, target_y, phi=target_phi)
                target_angles = [theta1, theta2, theta3]
                
                # Command target positions to internal motor actuators
                for i, joint_idx in enumerate(active_joints):
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angles[i],
                        force=150.0
                    )
                # Draw trailing paths in RED when tracking successfully
                line_color = [1, 0, 0]
                
            except ValueError:
                # Flash path YELLOW when circle or cursor extends past bounds
                line_color = [1, 1, 0]

            # 5. Project and Trace the True End-Effector Tip Position
            link_state = p.getLinkState(robot_id, end_effector_link_idx)
            wrist_pos = link_state[0]

            # Read raw joint states to sum the orientation relative to the horizon
            joint_states = p.getJointStates(robot_id, active_joints)
            t1 = joint_states[0][0]
            t2 = joint_states[1][0]
            t3 = joint_states[2][0]
            global_phi = t1 + t2 + t3

            # Project ahead by link length L3 to track the exact tip tool point
            tip_x = wrist_pos[0] + L3 * np.cos(global_phi)
            tip_y = wrist_pos[1] + L3 * np.sin(global_phi)
            tip_z = wrist_pos[2]
            current_tip_position = [tip_x, tip_y, tip_z]

            # Draw a 3D persistent debug trace line if a previous position exists
            if last_tip_position is not None:
                p.addUserDebugLine(
                    lineFromXYZ=last_tip_position,
                    lineToXYZ=current_tip_position,
                    lineColorRGB=line_color,
                    lineWidth=3.0,
                    lifeTime=12.0 # Slightly longer lifetime to view full circular lap
                )
            
            # Update tracking coordinate reference frames
            last_tip_position = current_tip_position

            p.stepSimulation()
            time.sleep(1.0 / 240.0)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    run_pybullet_simulation()
