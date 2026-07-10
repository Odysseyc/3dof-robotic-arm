# 3-DOF Robotic Arm

Independent robotics project exploring the design, control, and learning-based manipulation of a 3-DOF robotic arm platform.

## Overview

The goal of this project is to build and control a low-cost robotic arm while developing a deeper understanding of:

- Forward and inverse kinematics
- Robot modeling
- Embedded motor control
- Motion planning
- Simulation-to-real transfer
- Learning-based manipulation

## Project Goals

### Phase 1: Simulation
- Build a simulated 3-DOF robotic arm model
- Implement forward kinematics
- Implement inverse kinematics
- Develop trajectory planning methods

### Phase 2: Hardware Platform
- Design mechanical structure
- Select actuators and electronics
- Assemble physical prototype
- Implement low-level control

### Phase 3: Manipulation Learning
- Integrate vision-based control
- Explore imitation learning and reinforcement learning approaches
- Evaluate manipulation tasks

## Timeline

Summer 2026

- [ ] Robot design finalized
- [ ] Simulation environment created
- [ ] Kinematics implemented
- [ ] Hardware assembled
- [ ] Control system developed
- [ ] Manipulation experiments completed

## Cartesian Motion Demo

![Cartesian Motion](simulation/cartesian_motion.gif)

## Current Progress

Implemented forward kinematics for a planar 3-DOF robotic arm.

### Visualization

![Robot Arm](<docs/images/First robot pic.png>)

## Motion Planning Demo

The robotic arm computes inverse kinematics and generates smooth joint trajectories between configurations.

![Trajectory Demo](<docs/images/arm_animation.gif>)

## Control

The arm simulation includes PID joint control.

The controller models:
- proportional error correction
- integral accumulation
- derivative damping

Future work:
- rigid body dynamics
- torque limits
- physical hardware deployment

## Dynamics Simulation

The arm simulation now models:

- joint inertia
- damping
- torque inputs
- closed-loop PID control

Future extensions:
- gravity compensation
- rigid-body dynamics
- hardware deployment

## Motion Planning

The arm supports:

- joint-space trajectory interpolation
- Cartesian end-effector trajectories
- inverse kinematics conversion
- closed-loop tracking

Pipeline:

Cartesian target
→ IK
→ joint trajectory
→ controller
→ robot dynamics

## Technologies

- Python
- PyBullet
- NumPy
- ROS 2
- Embedded controllers
- Computer vision tools

## Author

Adam Sabet
