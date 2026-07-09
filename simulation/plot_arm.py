import numpy as np
import matplotlib.pyplot as plt

L1 = 0.15
L2 = 0.12
L3 = 0.10


theta1 = np.deg2rad(30)
theta2 = np.deg2rad(45)
theta3 = np.deg2rad(-20)

x0, y0 = 0, 0

x1 = L1*np.cos(theta1)
y1 = L1*np.sin(theta1)

x2 = x1 + L2*np.cos(theta1+theta2)
y2 = y1 + L2*np.sin(theta1+theta2)

x3 = x2 + L3*np.cos(theta1+theta2+theta3)
y3 = y2 + L3*np.sin(theta1+theta2+theta3)

plt.plot(
    [x0,x1,x2,x3],
    [y0,y1,y2,y3],
    "-o",
    linewidth=3,
)

plt.axis("equal")
plt.grid(True)
plt.show()
