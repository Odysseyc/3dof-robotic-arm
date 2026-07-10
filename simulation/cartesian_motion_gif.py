import sys 
import os 

sys.path.append(
  os.path.dirname(
    os.path.dirname(__file__)
  )
)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from control.forward_kinematics import forward_kinematics
from control.inverse_kinematics import inverse_kinematics
from control.trajectory import interpolate_cartesian


start = np.array([0.15, 0.10])
end = np.array([0.22, 0.15])

trajectory = interpolate_cartesian(
    start,
    end,
    steps=50
)


fig, ax = plt.subplots(figsize=(5,5))

ax.set_xlim(-0.4,0.4)
ax.set_ylim(-0.4,0.4)

ax.set_aspect("equal")

line, = ax.plot([], [], "o-", linewidth=3)
path, = ax.plot([], [], "r--", alpha=0.5)


positions = []


def update(frame):

    target = trajectory[frame]

    angles = inverse_kinematics(
        target[0],
        target[1]
    )

    points = [
        np.array([0,0]),
        forward_kinematics(
            angles[0],
            0,
            0
        ),
        forward_kinematics(
            angles[0],
            angles[1],
            0
        ),
        forward_kinematics(
            angles[0],
            angles[1],
            angles[2]
        )
    ]

    x = [p[0] for p in points]
    y = [p[1] for p in points]

    line.set_data(x,y)

    positions.append(points[-1])

    px = [p[0] for p in positions]
    py = [p[1] for p in positions]

    path.set_data(px,py)

    return line,path


animation = FuncAnimation(
    fig,
    update,
    frames=len(trajectory),
    interval=100,
    blit=True
)


animation.save(
    "cartesian_motion.gif",
    writer=PillowWriter(fps=10)
)

print("Saved cartesian_motion.gif")
