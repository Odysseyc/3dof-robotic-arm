import sys
import os

sys.path.append(
    os.path.dirname(
        os.path.dirname(__file__)
    )
)


from control.pid_controller import PIDController
from control.dynamics import JointDynamics


dt = 0.01


pid = PIDController(
    kp=10,
    ki=0,
    kd=1,
    dt=dt
)


dynamics = JointDynamics(
    mass=1,
    damping=0.5,
    dt=dt
)


target = 1.0

angle = 0
velocity = 0


for step in range(300):

    torque = pid.update(
        target,
        angle
    )


    angle, velocity = dynamics.update(
        angle,
        velocity,
        torque
    )


    if step % 30 == 0:
        print(
            f"time={step*dt:.2f}s "
            f"angle={angle:.3f}"
        )
