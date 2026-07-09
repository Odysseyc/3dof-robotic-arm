import numpy as np


def forward_kinematics(theta1, theta2, theta3):
    """
    Computes end-effector position for a simple 3-DOF planar arm.

    theta1: base rotation
    theta2: shoulder angle
    theta3: elbow angle

    Returns:
        x, y, z position
    """

    # Link lengths (meters)
    L1 = 0.3
    L2 = 0.25
    L3 = 0.15

    # Shoulder/elbow contribution
    r = (
        L1 * np.cos(theta2)
        + L2 * np.cos(theta2 + theta3)
        + L3 * np.cos(theta2 + theta3)
    )

    z = (
        L1 * np.sin(theta2)
        + L2 * np.sin(theta2 + theta3)
        + L3 * np.sin(theta2 + theta3)
    )

    # Base rotation
    x = r * np.cos(theta1)
    y = r * np.sin(theta1)

    return np.array([x, y, z])


if __name__ == "__main__":
    position = forward_kinematics(
        theta1=np.pi/4,
        theta2=np.pi/6,
        theta3=np.pi/3
    )

    print("End effector position:", position)