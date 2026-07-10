import numpy as np


def interpolate_cartesian(
    start_xy,
    end_xy,
    steps=100
):
    """
    Generates end-effector positions
    between two Cartesian points.
    """

    start_xy = np.array(start_xy)
    end_xy = np.array(end_xy)

    trajectory = []

    for alpha in np.linspace(0,1,steps):

        point = (
            (1-alpha)*start_xy
            + alpha*end_xy
        )

        trajectory.append(point)

    return np.array(trajectory)



if __name__ == "__main__":

    path = interpolate_cartesian(
        [0.15,0.10],
        [0.30,0.20],
        10
    )

    print(path)
