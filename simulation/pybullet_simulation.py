import sys
import os
import time
import numpy as np
import pybullet as p
import pybullet_data

# Ensure project root is in the path to read your custom IK module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.inverse_kinematics import inverse_kinematics

def run_pybullet_simulation():
    # 1. Initialize PyBullet Physics Client in GUI Mode
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Configure the viewer camera position
    p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.1, 0, 0.1])
    
    # 2. Build Environment
    p.loadURDF("plane.urdf")
    
    # Here we load a default industrial serial arm assembly model
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_path = os.path.join(project_root, "urdf", "arm_3dof.urdf")
    
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    
    active_joints = [0, 1, 2]
    
    # 3. Define the Target Path (Cartesian Space Coordinates)
    # Let's map a tracking path for the tip to execute
    target_x = 0.22
    target_y = 0.10
    target_phi = 0.0 # Our orientation variable relative to the horizon
    
    print(f"Resolving custom analytical IK for target destination: ({target_x}, {target_y})...")
    try:
        # Use your custom analytic 3-DOF IK function to obtain exact targets
        theta1, theta2, theta3 = inverse_kinematics(target_x, target_y, phi=target_phi)
        target_angles = [theta1, theta2, theta3]
    except ValueError as e:
        print(f"Kinematic bounds exception encountered: {e}")
        p.disconnect()
        return

    # 4. Simulation Execution Loop
    print("Beginning tracking runtime. Watch the PyBullet graphical scene layout.")
    for step in range(500):
        # Apply standard smooth target tracking over time
        if step < 200:
            # Interpolate gradually from home configuration up to targets
            fraction = step / 200.0
            current_targets = [angle * fraction for angle in target_angles]
        else:
            current_targets = target_angles

        # Command the explicit joint targets to PyBullet's position motor controllers
        for i, joint_idx in enumerate(active_joints):
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=current_targets[i],
                force=200.0  # Allow up to 200 Nm torque to overcome gravity forces
            )
            
        p.stepSimulation()
        time.sleep(1.0 / 240.0) # Maintain steady physics ticking updates at 240Hz
        
    print("Target coordinate reached and stabilized.")
    
    # Keep the window open indefinitely until you manually close it or hit Ctrl+C
    print("Simulation complete. Keeping GUI window open...")
    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    run_pybullet_simulation()
