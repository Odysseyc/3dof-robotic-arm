import numpy as np


def interpolate_joints(start_angles, end_angles, steps=100):
    """
    Linearly interpolate between two joint configurations.

    Parameters:
        start_angles: starting joint angles (radians)
        end_angles: target joint angles (radians)
        steps: number of intermediate positions

    Returns:
        Array of joint configurations
    """

    start_angles = np.array(start_angles)
    end_angles = np.array(end_angles)

    trajectory = []

    for alpha in np.linspace(0, 1, steps):
        angles = (1-alpha)*start_angles + alpha*end_angles
        trajectory.append(angles)

    return np.array(trajectory)


if __name__ == "__main__":

    start = np.array([0, 0, 0])
    end = np.array([1, 0.5, -1])

    path = interpolate_joints(start, end)

    print(path.shape)

def interpolate_cartesian(start, end, steps=100):
    """
    Linearly interpolate between Cartesian points.
    """

    start = np.array(start)
    end = np.array(end)

    trajectory = []

    for alpha in np.linspace(0,1,steps):
        point = (1-alpha)*start + alpha*end
        trajectory.append(point)

    return np.array(trajectory)
