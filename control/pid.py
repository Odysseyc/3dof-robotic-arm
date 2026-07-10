class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-100, 100)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, target, current, dt):
        error = target - current
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        
        self.last_error = error
        
        # Total output command
        output = p_term + i_term + d_term
        
        # Clamp torque output limits to protect the virtual actuators
        lower, upper = self.output_limits
        return max(lower, min(output, upper))

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
