import numpy as np
import matplotlib.pyplot as plt


targets = []
actuals = []


with open("tracking_results.txt", "r") as f:
    for line in f:
        if "Target:" in line:
            parts = line.strip().split("Actual:")
            
            target = parts[0].replace("Target:", "").strip()
            actual = parts[1].strip()

            target = np.fromstring(
                target.strip("[]"),
                sep=" "
            )

            actual = np.fromstring(
                actual.strip("[]"),
                sep=" "
            )

            targets.append(target)
            actuals.append(actual)


targets = np.array(targets)
actuals = np.array(actuals)


error = np.linalg.norm(
    targets - actuals,
    axis=1
)


plt.figure(figsize=(8,4))
plt.plot(error)

plt.xlabel("Trajectory Step")
plt.ylabel("Position Error (m)")
plt.title("Cartesian Tracking Error")

plt.grid()

plt.savefig(
    "tracking_error.png",
    dpi=300
)

plt.show()
