import numpy as np
import matplotlib.pyplot as plt

# Link lengths (mm)
L1 = 111.032
L2 = 138.361
L3 = 50.0  # Total length of the foot segment

# Desired ANKLE position (where L2 ends)
x_f = 0     #ANKLE target x
y_f = -200  #ANKLE target y
# x_f = 100
# y_f = -150 # Example for a different position

# Desired absolute foot orientation (relative to world X-axis)
# 0 radians = parallel to the ground
angle_foot_world_rad = np.radians(45) # Target 45 degrees

# ---------------------------------------------------------------
# --- Step 1 & 2: Calculate theta1 and theta2 for Ankle Position ---
# --- Using user's original "elbow-UP" calculation ---
# ---------------------------------------------------------------
d = np.hypot(x_f, y_f) # Distance hip to ankle

# Check reachability for ankle
min_reach = abs(L1 - L2)
max_reach = L1 + L2
if not (min_reach <= d <= max_reach):
    raise ValueError(f"Target Ankle ({x_f}, {y_f}) is unreachable. Distance {d:.2f} is outside range [{min_reach:.2f}, {max_reach:.2f}]")

# Compute the "elbow-UP" configuration (user's original method)
# Need to clip arguments for safety with floating point
phi_arg_t2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
phi_arg_t2 = np.clip(phi_arg_t2, -1.0, 1.0)
theta2 = -np.arccos(phi_arg_t2) 

alpha = np.arctan2(y_f, x_f) # Angle to target ankle

beta_arg_t1 = (d**2 + L1**2 - L2**2) / (2 * d * L1) # Internal angle at hip
beta_arg_t1 = np.clip(beta_arg_t1, -1.0, 1.0)
beta = np.arccos(beta_arg_t1)

theta1 = alpha + beta  # "+" uses elbow-up configuration

# ---------------------------------------------------------------
# --- Step 3: Calculate theta3 for Foot Orientation ---
# ---------------------------------------------------------------
angle_L2_world_rad = theta1 + theta2
raw_theta3 = angle_foot_world_rad - angle_L2_world_rad

# Normalize theta3 to be within -pi to +pi
theta3 = np.arctan2(np.sin(raw_theta3), np.cos(raw_theta3))


# ---------------------------------------------------------------
# --- Calculate Joint and End Positions ---
# ---------------------------------------------------------------
hip = np.array([0, 0])
knee = hip + np.array([L1 * np.cos(theta1), L1 * np.sin(theta1)])
# 'foot_ankle_pos' represents the ANKLE joint position (end of L2)
foot_ankle_pos = knee + np.array([L2 * np.cos(theta1 + theta2), L2 * np.sin(theta1 + theta2)])

# Calculate the absolute angle of the foot segment in the world frame
# This is the angle we use to position the heel and toe relative to the ankle
foot_segment_world_angle = theta1 + theta2 + theta3

# Calculate Heel position (L3/2 distance backwards from ankle along foot angle)
heel_pos = foot_ankle_pos + np.array([
    (L3 / 2) * np.cos(foot_segment_world_angle + np.pi), # Add pi for opposite direction
    (L3 / 2) * np.sin(foot_segment_world_angle + np.pi)
])

# Calculate Toe position (L3/2 distance forwards from ankle along foot angle)
toe_pos = foot_ankle_pos + np.array([
    (L3 / 2) * np.cos(foot_segment_world_angle),
    (L3 / 2) * np.sin(foot_segment_world_angle)
])


# --- Print calculated angles ---
print(f"\n--- 3-DOF IK Results (Elbow-Up Config) ---")
print(f"Target Ankle Position (x_f, y_f): ({x_f}, {y_f}) mm")
print(f"Target Foot Orientation: {np.degrees(angle_foot_world_rad):.3f} degrees")
print(f"Calculated Theta 1 (Hip):   {np.degrees(theta1):.3f} degrees ({theta1:.4f} radians)")
print(f"Calculated Theta 2 (Knee):  {np.degrees(theta2):.3f} degrees ({theta2:.4f} radians)")
print(f"Calculated Theta 3 (Ankle): {np.degrees(theta3):.3f} degrees ({theta3:.4f} radians)")
print(f"-----------------------------------------\n")


# --- Plotting ---
plt.figure(figsize=(7, 7)) # Slightly larger figure

# Plot L1 and L2 segments
plt.plot([hip[0], knee[0], foot_ankle_pos[0]], [hip[1], knee[1], foot_ankle_pos[1]], '-o', color='tab:blue', linewidth=4, markersize=8, label='L1 & L2')

# Plot L3 (foot segment) CENTERED around the ankle joint
plt.plot([heel_pos[0], toe_pos[0]], [heel_pos[1], toe_pos[1]], '-', color='tab:green', linewidth=4, label=f'L3 (Foot)')
# Also plot the ankle joint distinctly so it's clear it's the center
plt.plot(foot_ankle_pos[0], foot_ankle_pos[1], 'o', color='tab:red', markersize=9, label='Ankle Joint (Center of Foot)')


# Plot target marker for ANKLE position
plt.plot(x_f, y_f, 'rx', markersize=12, markeredgewidth=2, label='Target Ankle Position')

# Plot axes and grid
plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)
plt.axvline(0, color='gray', linestyle='--', linewidth=0.5)
plt.grid(True)

# Title and labels
plt.title(f"3-DOF Planar BDX Leg \nTarget Ankle:({x_f},{y_f}), Target Foot Angle: {np.degrees(angle_foot_world_rad):.1f}°")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")

# Settings
plt.axis('equal')
plt.legend(loc='best', fontsize='small') # Adjusted font size for potentially more items
plt.show()