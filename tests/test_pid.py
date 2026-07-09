import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(__file__)
    )
)

from control.pid_controller import PIDController


controller = PIDController(
    kp=2,
    ki=0.1,
    kd=0.05,
    dt=0.01
)


current_angle = 0
target_angle = 1.0


for i in range(10):

    command = controller.update(
        target_angle,
        current_angle
    )

    current_angle += command * 0.01

    print(
        f"Step {i}: angle={current_angle:.3f}"
    )
