import sys
import os
import time
import serial
import numpy as np

# Append project root to path to access control modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.inverse_kinematics import inverse_kinematics

class RobotHardwareInterface:
    def __init__(self, port='COM3', baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2) # Vital: Allow Arduino time to reset after opening serial port
            print(f"Successfully connected to Arduino on port {port}")
        except Exception as e:
            print(f"Failed to connect to serial port {port}: {e}")
            self.ser = None

    def radians_to_servo_degrees(self, theta1, theta2, theta3):
        """
        Converts raw kinematic radians into physical 0-180 servo angles.
        Assumes mechanical assembly has the servo horns centered (90 deg) at kinematic zero.
        """
        # Convert radians to degrees
        deg1 = np.degrees(theta1)
        deg2 = np.degrees(theta2)
        deg3 = np.degrees(theta3)
        
        # Map your kinematic ranges to physical 0-180 servo boundaries.
        # You will fine-tune these calibration offsets tomorrow during assembly.
        servo1 = int(90 + deg1)
        servo2 = int(90 + deg2)
        servo3 = int(90 + deg3)
        
        return servo1, servo2, servo3

    def send_target_angles(self, s1, s2, s3):
        """Transmits parsed servo angles over serial wire protocol."""
        if self.ser and self.ser.is_open:
            command = f"{s1},{s2},{s3}\n"
            self.ser.write(command.encode('utf-8'))
            
            # Read confirmation back from Arduino
            response = self.ser.readline().decode('utf-8').strip()
            return response
        return "Serial connection unavailable."

    def move_to_cartesian_target(self, x, y, z, phi=0.0):
        """Executes full pipeline: Cartesian (X,Y,Z) -> IK Radians -> Hardware Degrees -> Serial Write"""
        try:
            # 1. Compute analytical Inverse Kinematics from your control folder
            t1, t2, t3 = inverse_kinematics(x, y, z, phi)
            
            # 2. Map to physical servo domains
            s1, s2, s3 = self.radians_to_servo_degrees(t1, t2, t3)
            print(f"Target XYZ: ({x}, {y}, {z}) -> Servos: M1={s1}°, M2={s2}°, M3={s3}°")
            
            # 3. Stream to MCU
            ack = self.send_target_angles(s1, s2, s3)
            print(f"Arduino echo: {ack}")
            
        except ValueError as e:
            print(f"Kinematics Error: {e}")

    def close(self):
        if self.ser:
            self.ser.close()

if __name__ == "__main__":
    # Test script entry point
    # Change port designation to match your OS profile tomorrow
    arm = RobotHardwareInterface(port='/dev/ttyACM0', baudrate=115200)
    
    # Test safe movement target inside your workspace envelope
    # Given lengths: L1=0.15m, L2=0.12m, L3=0.10m
    print("\nTesting physical command stream to coordinate...")
    arm.move_to_cartesian_target(x=0.18, y=0.0, z=0.20, phi=0.0)
    
    arm.close()
