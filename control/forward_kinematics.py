import numpy as np


L1 = 0.15
L2 = 0.12
L3 = 0.10


def forward_kinematics(theta1, theta2, theta3):
    """
    Computes end-effector position of a planar 3-DOF arm.

    Angles are in radians.
    Lengths are in meters.
    """

    x = (
        L1*np.cos(theta1)
        + L2*np.cos(theta1+theta2)
        + L3*np.cos(theta1+theta2+theta3)
    )

    y = (
        L1*np.sin(theta1)
        + L2*np.sin(theta1+theta2)
        + L3*np.sin(theta1+theta2+theta3)
    )

    return np.array([x, y])


if __name__ == "__main__":

    position = forward_kinematics(
        np.deg2rad(30),
        np.deg2rad(45),
        np.deg2rad(-20),
    )

    print(position)
