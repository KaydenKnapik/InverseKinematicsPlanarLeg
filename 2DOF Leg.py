import numpy as np
import matplotlib.pyplot as plt

# Link lengths (mm)
L1 = 111.032
L2 = 138.361

# Desired foot position (adjust as needed)
x_f = 0
y_f = -200  # Foot is directly below hip

# Inverse kinematics
d = np.hypot(x_f, y_f)

if d > (L1 + L2) or d < abs(L1 - L2):
    raise ValueError("Target is unreachable.")

# Compute the "elbow-UP" configuration
theta2 = -np.arccos((d**2 - L1**2 - L2**2) / (2 * L1 * L2))  # NEGATIVE angle flips the knee
alpha = np.arctan2(y_f, x_f)
beta = np.arccos((d**2 + L1**2 - L2**2) / (2 * d * L1))
theta1 = alpha + beta  # "+" ins    tead of "-" to bend outward

# Joint positions
hip = np.array([0, 0])
knee = hip + np.array([L1 * np.cos(theta1), L1 * np.sin(theta1)])
foot = knee + np.array([L2 * np.cos(theta1 + theta2), L2 * np.sin(theta1 + theta2)])

print(theta1)
print(theta2)
# Plotting
plt.figure(figsize=(6, 6))
plt.plot([hip[0], knee[0], foot[0]], [hip[1], knee[1], foot[1]], '-o', linewidth=4, markersize=8)
plt.plot(x_f, y_f, 'rx', markersize=10, label='Target Foot Position')
plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)
plt.axvline(0, color='gray', linestyle='--', linewidth=0.5)
plt.title("2-DOF BDX Robot Leg")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
