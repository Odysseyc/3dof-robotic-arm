import numpy as np

from control.forward_kinematics import forward_kinematics
from control.inverse_kinematics import inverse_kinematics

target = (0.22, 0.10)

angles = inverse_kinematics(*target)

position = forward_kinematics(*angles)

print("Target:")
print(target)

print("\nJoint angles (degrees):")
print(np.degrees(angles))

print("\nRecovered position:")
print(position)

print("\nError:")
print(np.array(target) - position)
