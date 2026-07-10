import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(__file__)
    )
)

from control.dynamics import JointDynamics


robot = JointDynamics()


angle = 0
velocity = 0


for i in range(100):

    angle, velocity = robot.update(
        angle,
        velocity,
        torque=1.0
    )

    if i % 10 == 0:
        print(
            f"step={i}, angle={angle:.3f}, velocity={velocity:.3f}"
        )
