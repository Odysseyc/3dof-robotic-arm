import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(__file__)
    )
)

import numpy as np

from control.cartesian_trajectory import interpolate_cartesian
from control.inverse_kinematics import inverse_kinematics
from control.forward_kinematics import forward_kinematics

results = open("tracking_results.txt", "w")
path = interpolate_cartesian(
    [0.15, 0.10],
    [0.22, 0.15],
    steps=20
)


for point in path:

    angles = inverse_kinematics(
        point[0],
        point[1]
    )

    position = forward_kinematics(
        *angles
    )

    print(
        "Target:",
        point,
        "Actual:",
        position
    )
    
    results.write(
    f"Target: {point} Actual: {position}\n"
    )

results.close()
