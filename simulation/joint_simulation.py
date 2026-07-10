import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(__file__)
    )
)

import numpy as np

from control.pid_controller import PIDController


dt = 0.01

controller = PIDController(
    kp=5,
    ki=0,
    kd=0.5,
    dt=dt
)


target = np.pi/3

angle = 0
velocity = 0


for step in range(200):

    torque = controller.update(
        target,
        angle
    )

    velocity += torque * dt

    angle += velocity * dt


    if step % 20 == 0:
        print(
            f"time={step*dt:.2f}s angle={angle:.3f}"
        )
