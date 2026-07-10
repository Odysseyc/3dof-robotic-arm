import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(__file__)
    )
)

import numpy as np

from control.trajectory import interpolate_joints


trajectory = interpolate_joints(
    [0, 0, 0],
    [1, 0.5, -1],
    steps=5
)


print("Trajectory shape:")
print(trajectory.shape)


print("\nTrajectory:")
for point in trajectory:
    print(point)
