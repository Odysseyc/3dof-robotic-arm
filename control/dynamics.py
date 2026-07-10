import numpy as np


class JointDynamics:

    def __init__(
        self,
        mass=1.0,
        damping=0.1,
        dt=0.01
    ):
        self.mass = mass
        self.damping = damping
        self.dt = dt


    def update(
        self,
        angle,
        velocity,
        torque
    ):
        """
        Simple second-order joint dynamics:

        torque = inertia * acceleration
               + damping * velocity
        """

        acceleration = (
            torque
            - self.damping * velocity
        ) / self.mass


        velocity += acceleration * self.dt

        angle += velocity * self.dt


        return angle, velocity
