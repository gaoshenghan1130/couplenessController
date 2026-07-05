import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar

# ==========================================
# Common Parameter Settings
# ==========================================
V = 1.0          # Linear velocity (constant)
dt = 0.05        # Sampling time step
sim_time = 8.0  # Total simulation time
steps = int(sim_time / dt)
x_target = np.array([5.0, 5.0])  # Target position
time_axis = np.linspace(0, sim_time, steps)

# ==========================================
# 1. Optimization-Based Controller (Original)
# ==========================================
x_opt = np.array([0.0, 0.0, 0.0])  # Initial state [x, y, theta]
history_x_opt = np.zeros((steps, 3))
history_u3_opt = np.zeros(steps)

for t in range(steps):
    history_x_opt[t, :] = x_opt
    e_1 = x_target[0] - x_opt[0]
    e_2 = x_target[1] - x_opt[1]
    theta = x_opt[2]
    
    # Objective function for single-step local optimization
    def objective_function(u3):
        term1 = (e_1 + V * np.sin(theta) * u3) ** 2
        term2 = (e_2 - V * np.cos(theta) * u3) ** 2
        return term1 + term2
    
    res = minimize_scalar(objective_function, bounds=(-3.0, 3.0), method='bounded')
    u3 = res.x # type: ignore
    history_u3_opt[t] = u3
    
    # Kinematic update (Euler integration)
    x_opt[0] += V * np.cos(x_opt[2]) * dt
    x_opt[1] += V * np.sin(x_opt[2]) * dt
    x_opt[2] += u3 * dt

# ==========================================
# 2. PD Controller (Polar Coordinate Feedback)
# ==========================================
x_pd = np.array([0.0, 0.0, 0.0])  # Initial state [x, y, theta]
history_x_pd = np.zeros((steps, 3))
history_u3_pd = np.zeros(steps)

# PD Gain Tuning
Kp = 8.0  
Kd = 0.5  
prev_e_theta = 0.0

for t in range(steps):
    history_x_pd[t, :] = x_pd
    
    # Calculate the absolute target heading from current position
    dx = x_target[0] - x_pd[0]
    dy = x_target[1] - x_pd[1]
    target_theta = np.arctan2(dy, dx)
    
    # Calculate heading error and wrap it to [-pi, pi] to prevent over-rotation
    e_theta = target_theta - x_pd[2]
    e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
    
    # Calculate derivative term (rate of change of error)
    de_theta = (e_theta - prev_e_theta) / dt if t > 0 else 0.0
    prev_e_theta = e_theta
    
    # PD Control Law for angular velocity
    u3_pd = Kp * e_theta + Kd * de_theta
    
    # Enforce control input saturation bounds [-3.0, 3.0]
    u3_pd = np.clip(u3_pd, -3.0, 3.0)
    history_u3_pd[t] = u3_pd
    
    # Kinematic update (Euler integration)
    x_pd[0] += V * np.cos(x_pd[2]) * dt
    x_pd[1] += V * np.sin(x_pd[2]) * dt
    x_pd[2] += u3_pd * dt


# ==========================================
# 3. Plotting: Figure 1 - Optimization Results
# ==========================================
plt.figure(1, figsize=(14, 5))

# 2D Position Trajectory
plt.subplot(1, 2, 1)
plt.plot(history_x_opt[:, 0], history_x_opt[:, 1], '-b', linewidth=2, label='Opt Trajectory')
plt.plot(x_target[0], x_target[1], 'ro', markersize=8, label='Target Point')
plt.plot(0, 0, 'go', markersize=8, label='Start Point')
plt.title('Optimization-Based: 2D Position Trajectory', fontsize=12)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True, linestyle='--', alpha=0.6)
plt.axis('equal')
plt.legend()

# Heading and Control Input over Time
plt.subplot(1, 2, 2)
plt.plot(time_axis, history_x_opt[:, 2], 'g-', linewidth=2, label='Heading (theta)')
plt.plot(time_axis, history_u3_opt, 'r--', linewidth=1.5, label='Control Input (u3)')
plt.title('Optimization-Based: Heading & Control Over Time', fontsize=12)
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.tight_layout()


# ==========================================
# 4. Plotting: Figure 2 - PD Controller Results
# ==========================================
plt.figure(2, figsize=(14, 5))

# 2D Position Trajectory
plt.subplot(1, 2, 1)
plt.plot(history_x_pd[:, 0], history_x_pd[:, 1], '-r', linewidth=2, label='PD Trajectory')
plt.plot(x_target[0], x_target[1], 'ro', markersize=8, label='Target Point')
plt.plot(0, 0, 'go', markersize=8, label='Start Point')
plt.title('PD Controller: 2D Position Trajectory', fontsize=12)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True, linestyle='--', alpha=0.6)
plt.axis('equal')
plt.legend()

# Heading and Control Input over Time
plt.subplot(1, 2, 2)
plt.plot(time_axis, history_x_pd[:, 2], 'g-', linewidth=2, label='Heading (theta)')
plt.plot(time_axis, history_u3_pd, 'm--', linewidth=1.5, label='Control Input (u3)')
plt.title('PD Controller: Heading & Control Over Time', fontsize=12)
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.tight_layout()

# Display all active figures
plt.show()