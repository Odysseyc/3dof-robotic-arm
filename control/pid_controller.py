import numpy as np


class PIDController:

    def __init__(
        self,
        kp,
        ki,
        kd,
        dt
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.dt = dt

        self.integral = 0
        self.previous_error = 0


    def update(self, target, current):

        error = target - current

        self.integral += error * self.dt

        derivative = (
            error - self.previous_error
        ) / self.dt

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        self.previous_error = error

        return output
