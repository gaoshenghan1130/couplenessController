import pickle
import numpy as np
import sympy as sp
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

# ==========================================
# 1. Load Control Target Equations Exported by SymPy
# ==========================================
print("Loading symbolic equations...")
with open("./src/util/system_equations.pkl", "rb") as f_pkl:
    equations_data = pickle.load(f_pkl)

control_equations = equations_data['control_equations']

# Establish numerical mapping (Lambdify)
x1, x2, x3, x4 = sp.symbols('x1 x2 x3 x4', real=True)
m_rod, m_w, m_p, R, h, I, g, F_sym = sp.symbols('m_rod m_w m_p R h I g F', positive=True, real=True)
k = sp.symbols('k', positive=True, real=True)
e1, e2, e3, e4 = sp.symbols('e1 e2 e3 e4', real=True)

args_tuple = (F_sym, x1, x2, x3, x4, e1, e2, e3, e4, m_rod, m_w, m_p, R, h, I, g, k)
numerical_control_eqs = sp.lambdify(args_tuple, control_equations, modules='numpy')

# ==========================================
# 2. Define Control Optimizer Function
# ==========================================
def compute_control_F(current_state, current_error, p_vals, F_guess, weights=None):
    """
    Solve for the current control force F by minimizing the weighted residual 
    of the control equations using the least squares method.
    """
    if weights is None:
        # Default weights = 1.0 for all residual components
        weights = np.ones(4) 
        
    def cost_func(F_val):
        res = numerical_control_eqs(F_val[0], *current_state, *current_error, *p_vals)
        # Apply weighting to the residual vector element-wise
        return np.array(res).flatten() * weights
    
    # Restrict the control force within [-100.0, 100.0] N
    sol = least_squares(cost_func, [F_guess], bounds=(-100.0, 100.0)) 
    return sol.x[0]

# ==========================================
# 3. Define True Physical Dynamics of the System
# ==========================================
def system_dynamics(state, F, p):
    """
    The input state is defined based on the transformed coordinates:
    state = [x1, x2, x3, x4] = [theta, r - R*theta, dot_theta, dot_r - R*dot_theta]
    
    Returns derivatives: [dot_x1, dot_x2, dot_x3, dot_x4] 
                       = [dot_theta, dot_r - R*dot_theta, dot_dot_theta, dot_dot_r - R*dot_dot_theta]
    """
    x1_val, x2_val, x3_val, x4_val = state
    m_rod_v, m_w_v, m_p_v, R_v, h_v, I_v, g_v, _ = p
    
    # Recover original physical variable r from the transformed state for dynamics calculation
    theta = x1_val
    r = x2_val + R_v * theta
    u1 = x3_val  # i.e., dot_theta
    u2 = x4_val  # i.e., dot_r - R*dot_theta
    
    # 1. Denominator of the standard mass matrix (M11)
    M11 = m_w_v * R_v**2 + m_rod_v * r**2 + m_p_v * (R_v + h_v)**2 + I_v
    if abs(M11) < 1e-6: 
        M11 = 1e-6
        
    # 2. Calculate acceleration terms
    g0_3_num = -m_rod_v * g_v * r * np.cos(theta) + (m_w_v * g_v * R_v + m_p_v * g_v * (R_v + h_v)) * np.sin(theta) - m_rod_v * r * (2 * u2 * u1 + R_v * u1**2)
    
    dot_u1 = (F * R_v + g0_3_num) / M11
    dot_u2 = (F - m_rod_v * g_v * np.sin(theta) + m_rod_v * r * u1**2) / m_rod_v
    
    return np.array([u1, u2, dot_u1, dot_u2])

# ==========================================
# 4. Simulation Configuration & Parameter Assignment
# ==========================================
# --- Normalized physical system parameters ---
mass_at_end = 0.24                                      # Mass of the payload at the end of the rod (kg)
m_rod_v     = (0.15 + mass_at_end) * 2                     # Total mass of the pendulum rod (kg)
m_w_v       = 3.5                                       # Wheel mass (kg)
m_p_v       = 4.1                                       # Vehicle body/battery base mass (kg)
h_v         = 0.115                                     # Center of mass height offset (m)
R_v         = 0.2527                                    # Wheel radius (m)
g_v         = 9.81                                      # Gravitational acceleration (m/s^2)

# Moment of inertia calculation
I_b         = 0.012904182130007                         # Battery base X-axis moment of inertia
I_w         = 0.0459142776779452                        # Wheel X-axis moment of inertia
I_rod_calc  = 1/12 * 0.3 * (0.314**2) + 2 * mass_at_end * (0.157**2) # Calculated rod moment of inertia

I_v         = I_b + I_w + I_rod_calc                     
k_gain      = 1                                         # Control gain k

# Mapping to match the last 8 items of p_vals: (m_rod, m_w, m_p, R, h, I, g, k)
p_vals = (m_rod_v, m_w_v, m_p_v, R_v, h_v, I_v, g_v, k_gain)

# --- Optimizer Weight Configuration ---
# Customize these weights to prioritize penalizing specific control equation residuals.
# Format: [weight_eq1, weight_eq2, weight_eq3, weight_eq4]
error_weights = np.array([100.0, 50.0, 10.0, 10.0]) 

dt = 0.005          
t_max = 5.0       
n_steps = int(t_max / dt)

target_state = np.array([0.0, 0.0, 0.0, 0.0])

# Initial transformed state: [theta, r - R*theta, dot_theta, dot_r - R*dot_theta]
state = np.array([0.01 * np.pi/180, 0.0, 0.0, 0.0])

# Data logging initialization
time_history = np.linspace(0, t_max, n_steps)
state_history = np.zeros((n_steps, 4))
F_history = np.zeros(n_steps)

F_latest = 0.0 # Initial guess for control force

print("\n--- Initial State Control Equations Evaluation (Function of F) ---")
initial_error = state - target_state

subs_dict = {
    x1: state[0], x2: state[1], x3: state[2], x4: state[3],
    e1: initial_error[0], e2: initial_error[1], e3: initial_error[2], e4: initial_error[3],
    m_rod: m_rod_v, m_w: m_w_v, m_p: m_p_v,
    R: R_v, h: h_v, I: I_v, g: g_v, k: k_gain
}

print("Symbolic Equations with t=0 states and physical parameters plugged in:")
for idx, eq in enumerate(control_equations):
    eq_in_terms_of_F = eq.subs(subs_dict)
    
    # 使用 evalf(5) 保留5位小数，避免浮点数过长导致难以阅读
    print(f"\nEquation {idx+1}:")
    print(sp.expand(eq_in_terms_of_F).evalf(5))
print("--------------------------------------------------------------\n")

print("\nStarting continuous time simulation (Configured with Real System Params)...")
for i in range(n_steps):
    # 1. Calculate current control error: E = X - X_target
    current_error = state - target_state
    
    # 2. Solve for the optimal control force F at the current step
    F_latest = compute_control_F(state, current_error, p_vals, F_guess=F_latest, weights=error_weights)
    
    # Log current states and control force
    state_history[i, :] = state
    F_history[i] = F_latest
    
    # 3. State update using 4th-order Runge-Kutta method (RK4)
    k1 = system_dynamics(state, F_latest, p_vals)
    k2 = system_dynamics(state + dt/2 * k1, F_latest, p_vals)
    k3 = system_dynamics(state + dt/2 * k2, F_latest, p_vals)
    k4 = system_dynamics(state + dt * k3, F_latest, p_vals)
    state = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

print("Simulation finished successfully!")

# ==========================================
# 5. Plot Simulation Curves
# ==========================================
# Extract absolute physical variables (theta and r) for clean plotting
theta_all     = state_history[:, 0]                               
r_all         = state_history[:, 1] + R_v * state_history[:, 0]   
dot_theta_all = state_history[:, 2]                               
dot_r_all     = state_history[:, 3] + R_v * state_history[:, 2]   

plt.figure(figsize=(8, 6))

# Subplot 1: Absolute Angle (theta)
plt.subplot(3, 1, 1)
plt.plot(time_history, theta_all, label=r'$\theta$ (rad)', color='b')
plt.grid(True)
plt.ylabel('Angle')
plt.legend(loc='upper right')
plt.title(r'System Simulation (State: $x$, Output: $[\theta, r, \dot{\theta}, \dot{r}]^T$)')

# Subplot 2: Absolute Position (r)
plt.subplot(3, 1, 2)
plt.plot(time_history, r_all, label=r'$r$ (m)', color='r')
plt.grid(True)
plt.ylabel('Position')
plt.legend(loc='upper right')

# Subplot 3: Control Input Force (F)
plt.subplot(3, 1, 3)
plt.plot(time_history, F_history, label='Control Force F (N)', color='k', linestyle='--')
plt.grid(True)
plt.ylabel('Input Force')
plt.xlabel('Time (s)')
plt.legend(loc='upper right')

plt.tight_layout()
plt.show()