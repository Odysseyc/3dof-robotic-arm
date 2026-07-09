import numpy as np

L1 = 0.15
L2 = 0.12
L3 = 0.10


def inverse_kinematics(x, y):
    """
    Compute approximate joint angles for a planar 3DOF arm.

    For now, we treat the final link as always pointing forward,
    reducing the problem to a 2-link inverse kinematics problem.
    """

    # Effective wrist position
    x_wrist = x - L3
    y_wrist = y

    d = np.sqrt(x_wrist**2 + y_wrist**2)

    if d > L1 + L2:
        raise ValueError("Target is unreachable.")

    cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)

    theta2 = np.arccos(cos_theta2)

    theta1 = np.arctan2(y_wrist, x_wrist) - np.arctan2(
        L2*np.sin(theta2),
        L1 + L2*np.cos(theta2)
    )

    theta3 = -(theta1 + theta2)

    return theta1, theta2, theta3


if __name__ == "__main__":

    target = (0.22, 0.10)

    angles = inverse_kinematics(*target)

    print(np.degrees(angles))
