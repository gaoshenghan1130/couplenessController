import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar

# ==========================================
# 1. System Parameters & Initialization
# ==========================================
V = 1.0
L1 = 1.0
L2 = 1.0
dt = 0.01
time_steps = 1500

# Initial state [x, y, theta0, theta1, theta2]
x_start = np.array([0.0, 2.0, 1.5, 1.0, -1.2])
x_des = np.zeros(5)

# ----------------- Optimization Parameters -----------------
weights = np.array([0.0, 1.0, 1.0, 1.0])

# ----------------- PD Control Parameters -----------------
Kp = 0.5  # Proportional gain (targets lateral error y)
Kd = 1.5  # Derivative gain (targets heading theta0, approximating y_dot)

# ==========================================
# 2. Simulation 1: Optimization-based Control
# ==========================================
x_curr_opt = x_start.copy()
history_x_opt = [x_curr_opt.copy()]
history_u_opt = []

for t in range(time_steps):
    e = x_curr_opt - x_des
    e1, e2, e4, e5 = e[0], e[1], e[3], e[4]
    x3, x4, x5 = x_curr_opt[2], x_curr_opt[3], x_curr_opt[4]

    def cost_function(u3):
        # Predict future errors based on kinematics
        pred_e1 = V * np.sin(x3) * u3
        pred_e2 = -V * np.cos(x3) * u3
        pred_e4 = ((V**2 / (6 * L1**2)) * u3**3 - (V / L1) * np.cos(x3 - x4) * u3)
        pred_e5 = (-(V**2 * np.cos(x4 - x5) / (6 * L1 * L2)) * u3**3 
                   + (V / L2) * np.sin(x3 - x4) * np.sin(x4 - x5) * u3)

        res = np.array([e1 - pred_e1, e2 - pred_e2, e4 - pred_e4, e5 - pred_e5])
        return np.sum(weights * res**2)

    # Solve for optimal steering input within bounds
    opt_result = minimize_scalar(cost_function, bounds=(-3.0, 3.0), method='bounded')
    u3_opt = opt_result.x
    history_u_opt.append(u3_opt)

    # System kinematics update
    x_dot = np.array([
        V * np.cos(x3),
        V * np.sin(x3),
        u3_opt,
        (V / L1) * np.sin(x3 - x4),
        (V / L2) * np.cos(x3 - x4) * np.sin(x4 - x5)
    ])
    x_curr_opt = x_curr_opt + dt * x_dot
    history_x_opt.append(x_curr_opt.copy())

# ==========================================
# 3. Simulation 2: PD Control
# ==========================================
x_curr_pd = x_start.copy()
history_x_pd = [x_curr_pd.copy()]
history_u_pd = []

for t in range(time_steps):
    y = x_curr_pd[1]
    theta0 = x_curr_pd[2]
    theta1 = x_curr_pd[3]
    theta2 = x_curr_pd[4]

    # PD Control Law: drive tractor lateral error (y) and heading (theta0) to zero
    u3_pd = -Kp * y - Kd * theta0
    
    # Apply control saturation to match optimization bounds (-3.0 to 3.0)
    u3_pd = np.clip(u3_pd, -3.0, 3.0)
    history_u_pd.append(u3_pd)

    # System kinematics update
    x_dot = np.array([
        V * np.cos(theta0),
        V * np.sin(theta0),
        u3_pd,
        (V / L1) * np.sin(theta0 - theta1),
        (V / L2) * np.cos(theta0 - theta1) * np.sin(theta1 - theta2)
    ])
    x_curr_pd = x_curr_pd + dt * x_dot
    history_x_pd.append(x_curr_pd.copy())

# ==========================================
# 4. Data Processing & Coordinate Transformation
# ==========================================
def compute_positions(history_x):
    hx = np.array(history_x)
    tr_x, tr_y = hx[:, 0], hx[:, 1]
    
    # Trailer 1 hitch position
    tl1_x = tr_x - L1 * np.cos(hx[:, 3])
    tl1_y = tr_y - L1 * np.sin(hx[:, 3])
    
    # Trailer 2 hitch position
    tl2_x = tl1_x - L2 * np.cos(hx[:, 4])
    tl2_y = tl1_y - L2 * np.sin(hx[:, 4])
    
    return tr_x, tr_y, tl1_x, tl1_y, tl2_x, tl2_y, hx

x_opt = compute_positions(history_x_opt)
x_pd = compute_positions(history_x_pd)
time_axis = np.arange(time_steps) * dt

# ==========================================
# 5. Print Final Results
# ==========================================
print("\n========== Final State Comparison (Opt vs PD) ==========")
print(f"Tractor Y:      Opt = {x_opt[1][-1]:.4f}  |  PD = {x_pd[1][-1]:.4f}")
print(f"Heading (th0):  Opt = {x_opt[6][-1,2]:.4f}  |  PD = {x_pd[6][-1,2]:.4f}")
print(f"Trailer 1 Y:    Opt = {x_opt[3][-1]:.4f}  |  PD = {x_pd[3][-1]:.4f}")
print(f"Trailer 2 Y:    Opt = {x_opt[5][-1]:.4f}  |  PD = {x_pd[5][-1]:.4f}")
print(f"Final Control:  Opt = {history_u_opt[-1]:.4f}  |  PD = {history_u_pd[-1]:.4f}")
# ==========================================
# 6. Plotting (Modified)
# ==========================================
def plot_sim_figure(fig_num, title, x_data, u_data):
    fig, axs = plt.subplots(1, 2, figsize=(12, 5), num=fig_num)
    fig.suptitle(title, fontsize=16)
    
    tr_x, tr_y, tl1_x, tl1_y, tl2_x, tl2_y, hx = x_data
    
    axs[0].plot(tr_x, tr_y, linewidth=2, label="Tractor")
    axs[0].plot(tl1_x, tl1_y, "--", linewidth=2, label="Trailer 1")
    axs[0].plot(tl2_x, tl2_y, ":", linewidth=2, label="Trailer 2")
    axs[0].scatter(tr_x[0], tr_y[0], color="green", s=60, label="Start", zorder=5)
    axs[0].scatter(tr_x[-1], tr_y[-1], color="red", s=60, label="End", zorder=5)
    axs[0].axhline(0, color="gray", linestyle="--")
    axs[0].axis("equal")
    axs[0].grid(True)
    axs[0].set_title("Trajectories")
    axs[0].set_xlabel("X (m)")
    axs[0].set_ylabel("Y (m)")
    axs[0].legend()
    axs[1].set_ylim(-3.2, 5.2)

    axs[1].plot(time_axis, u_data, linewidth=2, color='purple')
    axs[1].grid(True)
    axs[1].set_title("Control Input ($u_3$)")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Steering Rate (rad/s)")
    axs[1].set_ylim(-3.2, 3.2)
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.88)

# Generate Figure 1: Optimization
plot_sim_figure(1, "Optimization Control Performance", x_opt, history_u_opt)

# Generate Figure 2: PD Control
plot_sim_figure(2, "PD Control Performance", x_pd, history_u_pd)

plt.show()

