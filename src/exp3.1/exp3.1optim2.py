import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar

# ==========================================
# Common Parameter Settings
# ==========================================
V = 1.0          # Linear velocity (constant)
dt = 0.05        # Sampling time step
sim_time = 10.0  # Total simulation time
steps = int(sim_time / dt)
x_target = np.array([5.0, 5.0])  # Target position
time_axis = np.linspace(0, sim_time, steps)

# ==========================================
# 1. Threshold-Based Optimization Controller
# ==========================================
x_opt = np.array([0.0, 0.0, 0.0])  # Initial state [x, y, theta]
history_x_opt = np.zeros((steps, 3))
history_u3_opt = np.zeros(steps)

epsilon = 1.1  # Small threshold to determine if e1 is close enough to 0

for t in range(steps):
    history_x_opt[t, :] = x_opt
    e_1 = x_target[0] - x_opt[0]
    e_2 = x_target[1] - x_opt[1]
    theta = x_opt[2]
    
    # Sequential Objective Function: prioritize e1, switch to e2 when e1 is small
    def objective_function(u3):
        if abs(e_1) < epsilon:
            return abs(e_2 - V * np.cos(theta) * u3)
        else:
            return abs(e_1 + V * np.sin(theta) * u3)
    
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




# ==========================================
# 3. Plotting: Figure 1 - Optimization Results
# ==========================================
plt.figure(1, figsize=(14, 5))

# 2D Position Trajectory
plt.subplot(1, 2, 1)
plt.plot(history_x_opt[:, 0], history_x_opt[:, 1], '-b', linewidth=2, label='Opt Trajectory')
plt.plot(x_target[0], x_target[1], 'ro', markersize=8, label='Target Point')
plt.plot(0, 0, 'go', markersize=8, label='Start Point')
plt.title('Threshold Optimization-Based: 2D Position Trajectory', fontsize=12)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True, linestyle='--', alpha=0.6)
plt.axis('equal')
plt.legend()

# Heading and Control Input over Time
plt.subplot(1, 2, 2)
plt.plot(time_axis, history_x_opt[:, 2], 'g-', linewidth=2, label='Heading (theta)')
plt.plot(time_axis, history_u3_opt, 'r--', linewidth=1.5, label='Control Input (u3)')
plt.title('Threshold Optimization-Based: Heading & Control Over Time', fontsize=12)
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.tight_layout()

plt.show()

