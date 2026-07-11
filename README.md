# Spatial 3-DOF Robotic Arm

An independent robotics project focused on building a fully integrated software and hardware stack for a 3-DOF robotic manipulator. This repository bridges analytical robot kinematics and real-time trajectory planning with industry-standard middleware (ROS 2), 3D visual telemetry (RViz), rigid-body physics engines (PyBullet), and embedded microcontroller hardware deployment.

---

## Features

- ✅ **Forward Kinematics:** Computes precise spatial link positions.
- ✅ **Analytical Inverse Kinematics Solver:** Solves exact geometric joint angles from 3D Cartesian target positions.
- ✅ **PyBullet Physics Simulation:** Validates torques, contact forces, and gravity compensation in a rigid-body physics environment.
- ✅ **ROS 2 Humble Integration:** Native ROS 2 nodes streaming joint and coordinate data across DDS.
- ✅ **URDF Robot Modeling:** Physical robot description including mass, inertia, collision, and visual geometry.
- ✅ **3D Visualization with RViz:** Live 30 Hz visualization of robot motion and coordinate transforms.
- ✅ **Interactive Motion Planning:** Continuous trajectory generation including vertical circular paths.
- ✅ **Closed-Loop Simulation:** Matplotlib visualization, PyBullet validation, and PID controller testing.
- ✅ **Embedded Hardware Deployment:** Arduino UNO R4 WiFi integration with MG996R high-torque servos.

---

# Project Structure

```text
3dof-robotic-arm/
├── arm_control/
│   ├── arm_control/
│   │   ├── joint_publisher.py      # Waving test generator
│   │   ├── ik_tracker.py           # 3D inverse kinematics node
│   │   └── circle_planner.py       # Continuous trajectory planner
│   │
│   ├── launch/
│   │   └── visualize_arm.launch.py
│   │
│   ├── urdf/
│   │   └── arm_3dof.urdf
│   │
│   ├── package.xml
│   └── setup.py
│
├── control/                        # Kinematics, controllers, planners
├── simulation/                     # PyBullet & visualization scripts
├── tests/
├── docs/
└── README.md
```

---

# Motion Planning Pipeline

```text
        [ 3D Cartesian Goal Path ]
        (Continuous Circle Planner)
                    │
                    ▼
         [ geometry_msgs/msg/Point ]
                    │
                    ▼
       [ 3D Geometric Inverse Kinematics ]
               (ik_tracker Node)
                    │
                    ▼
         [ sensor_msgs/msg/JointState ]
                    │
     ┌──────────────┼──────────────┐
     ▼              ▼              ▼
[ ROS 2 Utilities ] [ PyBullet Sim ] [ Embedded Controller ]
(state_publisher)   (Physics Engine)  (Arduino R4 Serial)
     │              │              │
     ▼              ▼              ▼
 [ 3D Telemetry ]   [ Torque/Load ]   [ Physical Actuation ]
  (RViz Visuals)    (Force Feedback)   (MG996R Servo PWM)
```

---

# Current Results

## 3D ROS 2 Telemetry & Joint Tracking

The software pipeline converts continuous Cartesian trajectories into joint-space commands through analytical inverse kinematics, broadcasting real-time transforms for visualization inside RViz while simultaneously validating the same commands in PyBullet.

### Robot Visualization (Legacy Simulation)

![Robot Arm](docs/images/First%20robot%20pic.png)

### Joint Motion Animation

![Joint Motion](docs/images/arm_animation.gif)

### Cartesian Motion Demo

![Cartesian Motion](simulation/cartesian_motion.gif)

---

# Implemented Components

## Kinematics & Modeling

- Forward kinematics for a spatial 3-link manipulator.
- Analytical inverse kinematics using explicit geometric equations.
- Complete URDF robot model with:
  - Visual geometry
  - Collision geometry
  - Inertial properties
  - Colored links
  - Link dimensions:

| Link | Length |
|------|--------|
| L₁ | 5 cm |
| L₂ | 15 cm |
| L₃ | 12 cm |
| L₄ | 10 cm |

---

## Physics Simulation (PyBullet)

- Direct URDF import into the physics engine.
- Rigid-body dynamics simulation.
- Gravity validation.
- Torque estimation.
- Collision detection.
- Contact force analysis.
- Continuous closed-loop validation against analytical control algorithms.

---

## ROS 2 Architecture

- Distributed ROS 2 nodes communicating through DDS.
- Ubuntu 22.04 LTS (WSL2) development environment.
- Launch files for one-command startup.
- Automatic TF broadcasting.
- RViz visualization.
- Software rendering compatibility using:

```bash
LIBGL_ALWAYS_SOFTWARE=1
```

---

## Motion Planning & Control

- Static point-to-point targeting.
- Continuous Cartesian trajectory generation.
- Vertical circular path planning.
- Real-time 30 Hz execution.
- Joint-space interpolation.

---

# Technologies & Hardware

## Software

| Category | Technology |
|-----------|------------|
| Operating System | Linux Mint / Ubuntu 22.04 LTS (WSL2) |
| Middleware | ROS 2 Humble |
| Physics Engine | PyBullet |
| Visualization | RViz2 |
| Languages | Python, C++ |
| Libraries | NumPy, Matplotlib, Pillow |

---

## Hardware (Active Prototyping)

| Component | Hardware |
|-----------|----------|
| Microcontroller | Arduino UNO R4 WiFi |
| CPU | 32-bit ARM Cortex-M4 |
| Servos | 4× MG996R Metal Gear Servos |
| Power Supply | Arkare 5V 4A External DC Supply |
| Prototyping | Barrel jack adapters, solderless breadboards, jumper wires |

---

# Future Work

## Physical Assembly

- Complete CAD model development in Onshape.
- Servo mounting design.
- Structural arm component fabrication.
- 3D printing and assembly.

## Firmware

- Arduino firmware implementation.
- Serial or micro-ROS communication.
- Joint command parsing.
- PWM servo control.

## Autonomous Robotics

- Reinforcement learning integration.
- Vision-guided manipulation.
- Autonomous reaching.
- Sim-to-real transfer using PyBullet.

---

# Author

**Adam Sabet**
