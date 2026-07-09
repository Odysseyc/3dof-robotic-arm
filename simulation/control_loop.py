import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(__file__)
    )
)

import numpy as np

from control.forward_kinematics import forward_kinematics
from control.inverse_kinematics import inverse_kinematics
from control.controller import position_controller


target = np.array([0.25,0.10])


angles = inverse_kinematics(
    target[0],
    target[1]
)


position = forward_kinematics(
    *angles
)


command = position_controller(
    position,
    target
)


print("Joint angles:")
print(angles)

print("\nCurrent position:")
print(position)

print("\nController output:")
print(command)
