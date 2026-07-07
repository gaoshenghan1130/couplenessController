import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# ================= 1. 系统物理参数 =================
g = 9.81
L = 1.0
dt = 0.02       # 物理仿真步长
sim_time = 5.0  # 仿真总时长

# ================= 2. 控制器参数设计 =================
# 误差收敛增益 K (类似 PD 参数，决定我们期望的 error vector 大小)
# 分别对应 [位置 x1, 角度 x2, 速度 x3, 角速度 x4]
K = np.array([1.0, 1.0, 1.0, 1.0]) 

# 优化目标权重 W (决定优化器在拟合 dx_total 时更看重哪个维度)
# 倒立摆系统中，角度 x2 和角速度 x4 的拟合优先级必须最高
W = np.array([0.0, 10.0, 1.0, 5.0])

# ================= 3. 带三阶项的非线性 PD 目标函数 =================
def cost_function(u_array, state):
    u = u_array[0]
    x1, x2, x3, x4 = state
    
    # --- A. 计算几何曲率 K2 和 K4 ---
    K2 = -(2 * x4 * np.sin(x2)) / L - (g * np.cos(x2)**2) / (L**2)
    K4 = (x4**2 * np.cos(x2)) / L - (g * x4 * np.sin(2*x2)) / (L**2) - (g * np.cos(2*x2)) / (L**2)
    
    # --- B. 计算控制产生的纯时空演化量 dx_total (代入 dt=1 的简化结果) ---
    dx1_tot = u
    
    term_inner = 1.0 + (1/6) * K2 * (-u * np.cos(x2) / L)**2
    dx2_tot = (1/6) * K2 * (u**3) - (u * np.cos(x2) / L) * term_inner
    
    dx3_tot = u
    
    dx4_tot = (1/6) * K4 * (u**3) - (u * np.cos(x2) / L)
    
    # --- C. 计算目标误差向量 e_error ---
    e_err = -K * state
    
    # --- D. 计算加权平方和误差 ---
    J = W[0] * (e_err[0] - dx1_tot)**2 + \
        W[1] * (e_err[1] - dx2_tot)**2 + \
        W[2] * (e_err[2] - dx3_tot)**2 + \
        W[3] * (e_err[3] - dx4_tot)**2
        
    return J

# ================= 4. 仿真主循环 =================
state = np.array([0.0, 0.1, 0.0, 0.0]) 

history_state = []
history_u = []
time_steps = np.arange(0, sim_time, dt)
u_guess = [0.0]


for t in time_steps:
    history_state.append(state.copy())
    
    # 求解 min || W * (e_err - dx_tot) ||^2
    res = minimize(cost_function, u_guess, args=(state,), method='BFGS')
    u_opt = - res.x[0]
    
    u_opt = np.clip(u_opt, -50.0, 50.0) 
    
    history_u.append(u_opt)
    u_guess = [u_opt] # 热启动下一帧
    
    x1, x2, x3, x4 = state
    state[0] += x3 * dt
    state[1] += x4 * dt
    state[2] += u_opt * dt
    state[3] += (g * np.sin(x2) - u_opt * np.cos(x2)) / L * dt

history_state = np.array(history_state)
history_u = np.array(history_u)

# ================= 5. 可视化结果 =================
plt.figure(figsize=(8, 6))

plt.subplot(4, 1, 1)
plt.plot(time_steps, history_state[:, 0], label='Cart Position (x1)', color='purple')
plt.ylabel('Pos (m)')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(time_steps, history_state[:, 1], label='Pendulum Angle (x2)', color='red', linewidth=2)
plt.axhline(0, color='black', linestyle='--')
plt.ylabel('Angle (rad)')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(time_steps, history_state[:, 3], label='Angular Velocity (x4)', color='orange')
plt.ylabel('Ang Vel (rad/s)')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(time_steps, history_u, label='Control Input (u)', color='blue')
plt.ylabel('Accel (m/s^2)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()