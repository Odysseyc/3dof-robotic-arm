import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(__file__)
    )
)

from control.cartesian_trajectory import interpolate_cartesian


trajectory = interpolate_cartesian(
    [0.15,0.10],
    [0.30,0.20],
    steps=5
)


print(trajectory)
