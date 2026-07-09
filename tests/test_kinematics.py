import sys
import os
import numpy as np

# Add project root to Python path
sys.path.append(
    os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )
)

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
