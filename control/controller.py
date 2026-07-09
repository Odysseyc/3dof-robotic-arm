import numpy as np


def position_controller(current, target, gain=0.5):
    """
    Simple proportional controller.

    current: current end-effector position [x,y]
    target: desired position [x,y]

    Returns velocity command.
    """

    error = np.array(target) - np.array(current)

    velocity = gain * error

    return velocity


if __name__ == "__main__":

    current_position = np.array([0.15, 0.10])
    target_position = np.array([0.25, 0.15])

    command = position_controller(
        current_position,
        target_position
    )

    print("Movement command:", command)
