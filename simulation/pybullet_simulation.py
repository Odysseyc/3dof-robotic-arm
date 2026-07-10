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
    
    # --- CHANGED: Angled 3D Isometric Viewport Camera Perspective ---
    p.resetDebugVisualizerCamera(
        cameraDistance=0.65, 
        cameraYaw=45, 
        cameraPitch=-30, 
        cameraTargetPosition=[0.0, 0.0, 0.1]
    )
    p.loadURDF("plane.urdf")
    
    # 2. Load Custom 3-DOF Robot Arm (Now configured for 3D)
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_path = os.path.join(project_root, "urdf", "arm_3dof.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    active_joints = [0, 1, 2]
    
    # CRITICAL for Torque Control: Disable internal gear position actuators
    for joint_idx in active_joints:
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_idx,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0,
            force=0
        )
    
    # Load parameters from configuration file
    config = load_robot_config()
    L1 = config['robot']['link_lengths']['L1']  # Vertical pedestal height
    L2 = config['robot']['link_lengths']['L2']
    L3 = config['robot']['link_lengths']['L3']
    max_reach = L1 + L2 + L3

    # Pull PID gains and instantiate controller wrappers if needed
    from control.pid import PIDController
    kp, ki, kd = config['control']['kp'], config['control']['ki'], config['control']['kd']
    pid_controllers = [
        PIDController(kp=kp, ki=ki, kd=kd, output_limits=(-50, 50)),
        PIDController(kp=kp, ki=ki, kd=kd, output_limits=(-50, 50)),
        PIDController(kp=kp, ki=ki, kd=kd, output_limits=(-50, 50))
    ]

    # --- CHANGED: 3D Interactive GUI Sliders ---
    slider_mode = p.addUserDebugParameter("AUTO MODE (0=Off, 1=On)", 0, 1, 1)
    slider_x = p.addUserDebugParameter("Target X", -max_reach, max_reach, 0.18)
    slider_y = p.addUserDebugParameter("Target Y", -max_reach, max_reach, 0.12)
    slider_z = p.addUserDebugParameter("Target Z (Height)", 0.0, max_reach, 0.22) # Safe height slider
    slider_phi = p.addUserDebugParameter("Target Pitch (Phi)", -np.pi/2, np.pi/2, 0.0)

    # Tracking paths variables
    end_effector_link_idx = 2
    last_tip_position = None
    
    # 3D Auto-Trajectory Variables (Traces a tilted circle floating in the air)
    circle_center_x, circle_center_y, circle_center_z = 0.16, 0.12, 0.20
    circle_radius = 0.05
    sim_time = 0.0
    dt = 1.0 / 240.0

    # Matplotlib Setup
    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 4))
    fig.canvas.manager.set_window_title("Real-Time Telemetry: 3D Tracking Error")
    time_history, error_history = [], []
    line, = ax.plot([], [], 'b-', linewidth=2, label="3D Euclidean Error (m)")
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 0.05)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (meters)")
    ax.grid(True)
    start_wall_time = time.time()

    print("\n=======================================================")
    print("SPATIAL 3D WORKSPACE INTERFACE ONLINE!")
    print("Move sliders or watch the arm swing and lift through space.")
    print("=======================================================\n")

    # 3. Core Loop
    try:
        while p.isConnected() and plt.fignum_exists(fig.number):
            auto_mode_active = p.readUserDebugParameter(slider_mode) >= 0.5
            target_phi = p.readUserDebugParameter(slider_phi)
            
            if auto_mode_active:
                sim_time += dt
                speed_factor = 2.5
                # Parametric equations defining a 3D flying circle pattern
                target_x = circle_center_x + circle_radius * np.cos(speed_factor * sim_time)
                target_y = circle_center_y + circle_radius * np.sin(speed_factor * sim_time)
                target_z = circle_center_z + 0.02 * np.cos(speed_factor * sim_time) # Smoothly waving height
            else:
                target_x = p.readUserDebugParameter(slider_x)
                target_y = p.readUserDebugParameter(slider_y)
                target_z = p.readUserDebugParameter(slider_z)
                sim_time = 0.0
            
            line_color = [1, 0, 0] # Red trace line default
            
            try:
                # 1. Resolve joint angles using your upgraded 3D solver
                theta1, theta2, theta3 = inverse_kinematics(target_x, target_y, target_z, phi=target_phi)
                target_angles = [theta1, theta2, theta3]
                
                # 2. Get actual system states
                joint_states = p.getJointStates(robot_id, active_joints)
                
                # 3. Compute torque control using PID loops
                for i, joint_idx in enumerate(active_joints):
                    current_angle = joint_states[i][0]
                    calculated_torque = pid_controllers[i].compute(
                        target=target_angles[i], 
                        current=current_angle, 
                        dt=dt
                    )
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.TORQUE_CONTROL,
                        force=calculated_torque
                    )
            except ValueError:
                line_color = [1, 1, 0] # Yellow line if targets break boundary envelope limits

            # --- 4. CHANGED: Compute the TRUE End-Effector Tip Position in 3D Space ---
            # Extract wrist position from PyBullet
            link_state = p.getLinkState(robot_id, end_effector_link_idx)
            wrist_pos = link_state[0] # (x_w, y_w, z_w)

            # Get current joint angles to run spatial forward kinematics projection
            joint_states = p.getJointStates(robot_id, active_joints)
            t1 = joint_states[0][0] # Base waist rotation (yaw around Z)
            t2 = joint_states[1][0] # Shoulder pitch around Y
            t3 = joint_states[2][0] # Elbow pitch around Y
            
            # Link 3's net pitch angle relative to the ground horizon plane
            pitch_net = t2 + t3 
            
            # Project link 3 out along its yaw and pitch vectors to find the true tool point
            tip_x = wrist_pos[0] + L3 * np.cos(pitch_net) * np.cos(t1)
            tip_y = wrist_pos[1] + L3 * np.cos(pitch_net) * np.sin(t1)
            tip_z = wrist_pos[2] + L3 * np.sin(pitch_net)
            current_tip_position = [tip_x, tip_y, tip_z]

            # 5. Telemetry Tracking
            error = np.sqrt((target_x - tip_x)**2 + (target_y - tip_y)**2 + (target_z - tip_z)**2)
            current_wall_time = time.time() - start_wall_time
            time_history.append(current_wall_time)
            error_history.append(error)
            
            if len(time_history) > 2400:
                time_history.pop(0)
                error_history.pop(0)

            if int(current_wall_time * 240) % 5 == 0:
                line.set_data(time_history, error_history)
                if current_wall_time > 10:
                    ax.set_xlim(current_wall_time - 10, current_wall_time)
                if max(error_history) > ax.get_ylim()[1]:
                    ax.set_ylim(0, max(error_history) * 1.2)
                fig.canvas.draw()
                fig.canvas.flush_events()

            # Render 3D path trace segments
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
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()
        plt.close('all')

if __name__ == "__main__":
    run_pybullet_simulation()
