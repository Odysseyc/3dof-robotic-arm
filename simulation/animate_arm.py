import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Add project root to Python path
sys.path.append(
    os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )
)

from control.forward_kinematics import forward_kinematics
from control.inverse_kinematics import inverse_kinematics
from control.trajectory import interpolate_joints


L1 = 0.15
L2 = 0.12
L3 = 0.10


def get_points(angles):

    theta1, theta2, theta3 = angles

    x0, y0 = 0, 0

    x1 = L1*np.cos(theta1)
    y1 = L1*np.sin(theta1)

    x2 = x1 + L2*np.cos(theta1+theta2)
    y2 = y1 + L2*np.sin(theta1+theta2)

    x3 = x2 + L3*np.cos(theta1+theta2+theta3)
    y3 = y2 + L3*np.sin(theta1+theta2+theta3)

    return [x0,x1,x2,x3], [y0,y1,y2,y3]


# Starting and ending poses

start = np.array([0,0,0])

target_position = (0.22,0.10)

goal = np.array(
    inverse_kinematics(*target_position)
)


trajectory = interpolate_joints(
    start,
    goal,
    steps=100
)


fig, ax = plt.subplots()

ax.set_xlim(-0.4,0.4)
ax.set_ylim(-0.4,0.4)
ax.set_aspect("equal")
ax.grid()


line, = ax.plot([], [], "-o", linewidth=3)


def update(frame):

    x, y = get_points(
        trajectory[frame]
    )

    line.set_data(x,y)

    return line,


animation = FuncAnimation(
    fig,
    update,
    frames=len(trajectory),
    interval=50,
    blit=True
)


plt.show()
