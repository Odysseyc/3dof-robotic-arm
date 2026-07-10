import sys
import os
import time
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt

# Ensure project root is in the path to read your custom modules
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
    p.loadURDF("plane.urdf")
    
    # 2. Load Custom 3-DOF Robot Arm
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_path = os.path.join(project_root, "urdf", "arm_3dof.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    active_joints = [0, 1, 2]
    
    # Load config parameters
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']
    L2 = config['robot']['link_lengths']['L2']
    L3 = config['robot']['link_lengths']['L3']
    max_reach = L1 + L2 + L3

    # 3. Initialize PyBullet Interactive GUI Sliders
    slider_mode = p.addUserDebugParameter("AUTO MODE (0=Off, 1=On)", 0, 1, 1) # Default to auto circle
    slider_x = p.addUserDebugParameter("Target X (Manual)", -max_reach, max_reach, 0.20)
    slider_y = p.addUserDebugParameter("Target Y (Manual)", -max_reach, max_reach, 0.15)
    slider_phi = p.addUserDebugParameter("Target Phi (Rad)", -np.pi, np.pi, 0.0)

    # Tracking variables
    end_effector_link_idx = 2
    last_tip_position = None
    
    # Parametric circle variables
    circle_center_x, circle_center_y, circle_radius = 0.18, 0.12, 0.06
    sim_time = 0.0

    # --- 4. Initialize Live Matplotlib Setup ---
    plt.ion() # Turn on interactive/non-blocking mode
    fig, ax = plt.subplots(figsize=(6, 4))
    fig.canvas.manager.set_window_title("Real-Time Telemetry: Tracking Error")
    
    time_history = []
    error_history = []
    
    line, = ax.plot([], [], 'r-', linewidth=2, label="Euclidean Error (m)")
    ax.set_xlim(0, 10)     # Show a rolling 10-second window
    ax.set_ylim(0, 0.02)   # 0 to 2cm error scale (adjust if needed)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (meters)")
    ax.grid(True)
    ax.legend(loc="upper right")
    
    start_wall_time = time.time()

    print("\n=======================================================")
    print("Live Telemetry Streaming Active!")
    print("Keep both the PyBullet and Matplotlib windows visible.")
    print("=======================================================\n")

    # 5. Infinite Real-Time Control Loop
    try:
        while p.isConnected() and plt.fignum_exists(fig.number):
            auto_mode_active = p.readUserDebugParameter(slider_mode) >= 0.5
            target_phi = p.readUserDebugParameter(slider_phi)
            
            if auto_mode_active:
                sim_time += 1.0 / 240.0
                speed_factor = 2.0
                target_x = circle_center_x + circle_radius * np.cos(speed_factor * sim_time)
                target_y = circle_center_y + circle_radius * np.sin(speed_factor * sim_time)
            else:
                target_x = p.readUserDebugParameter(slider_x)
                target_y = p.readUserDebugParameter(slider_y)
                sim_time = 0.0
            
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
                line_color = [1, 0, 0]
            except ValueError:
                line_color = [1, 1, 0]

            # 6. Calculate True End-Effector Tip Position
            link_state = p.getLinkState(robot_id, end_effector_link_idx)
            wrist_pos = link_state[0]

            joint_states = p.getJointStates(robot_id, active_joints)
            t1, t2, t3 = joint_states[0][0], joint_states[1][0], joint_states[2][0]
            global_phi = t1 + t2 + t3

            tip_x = wrist_pos[0] + L3 * np.cos(global_phi)
            tip_y = wrist_pos[1] + L3 * np.sin(global_phi)
            tip_z = wrist_pos[2]
            current_tip_position = [tip_x, tip_y, tip_z]

            # 7. Calculate Error Metrics
            # Euclidean distance between targeted command coordinates and physical output
            error = np.sqrt((target_x - tip_x)**2 + (target_y - tip_y)**2)
            current_wall_time = time.time() - start_wall_time

            # 8. Append data and maintain a rolling window buffer
            time_history.append(current_wall_time)
            error_history.append(error)
            
            # Keep only the last 10 seconds of data (240Hz * 10s = ~2400 points)
            if len(time_history) > 2400:
                time_history.pop(0)
                error_history.pop(0)

            # 9. Dynamic Matplotlib Update (Every 5 frames to maximize FPS)
            if int(current_wall_time * 240) % 5 == 0:
                line.set_data(time_history, error_history)
                
                # Roll the X-axis window forward dynamically over time
                if current_wall_time > 10:
                    ax.set_xlim(current_wall_time - 10, current_wall_time)
                
                # Dynamically autoscale Y-axis if error spikes during initialization or out-of-bounds
                if max(error_history) > ax.get_ylim()[1]:
                    ax.set_ylim(0, max(error_history) * 1.2)
                
                fig.canvas.draw()
                fig.canvas.flush_events()

            # 10. Draw Trace Lines
            if last_tip_position is not None:
                p.addUserDebugLine(
                    lineFromXYZ=last_tip_position,
                    lineToXYZ=current_tip_position,
                    lineColorRGB=line_color,
                    lineWidth=3.0,
                    lifeTime=10.0
                )
            
            last_tip_position = current_tip_position
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()
        plt.close('all')

if __name__ == "__main__":
    run_pybullet_simulation()
