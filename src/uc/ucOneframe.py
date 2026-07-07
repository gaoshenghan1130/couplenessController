import pickle
import numpy as np
import sympy as sp
from scipy.optimize import root

# ==========================================
# 1. 重新定义需要映射的符号变量 (为了 lambdify)
# ==========================================
x1, x2, x3, x4 = sp.symbols('x1 x2 x3 x4', real=True)
m_rod, m_w, m_p, R, h, I, g, F = sp.symbols('m_rod m_w m_p R h I g F', positive=True, real=True)
k = sp.symbols('k', positive=True, real=True)
e1, e2, e3, e4 = sp.symbols('e1 e2 e3 e4', real=True)

# 集合所有参数
params_symbols = (m_rod, m_w, m_p, R, h, I, g, k)
state_symbols = (x1, x2, x3, x4)
error_symbols = (e1, e2, e3, e4)

# ==========================================
# 2. 导入导出的符号方程
# ==========================================
print("Loading symbolic equations...")
with open("./src/util/system_equations.pkl", "rb") as f_pkl:
    equations_data = pickle.load(f_pkl)

control_equations = equations_data['control_equations']

# ==========================================
# 3. 将 SymPy 表达式转化为快速的数值计算函数
# ==========================================
print("Converting symbolic equations to numerical functions (lambdify)...")

# 这里的输入顺序：控制量F, 状态X, 误差E, 系统参数
# 输出是一个 4x1 的列向量，我们将其平铺为一维数组
args_tuple = (F, *state_symbols, *error_symbols, *params_symbols)

# 使用 lambdify 生成高阶函数
numerical_control_eqs = sp.lambdify(args_tuple, control_equations, modules='numpy')

def cost_function_for_F(F_val, current_state, current_error, current_params):
    F_scalar = F_val[0]
    res = numerical_control_eqs(F_scalar, *current_state, *current_error, *current_params)
    
    # 原始残差向量 (4x1)
    raw_residuals = np.array(res).flatten()
    
    # === 新增：定义权重向量 ===
    # 让优化器更关注前两个状态 (x1, x2) 的误差，对后两个状态 (x3, x4) 宽容一点
    weights = np.array([1.0, 1.0, 1.0, 1.0]) 
    
    # 返回加权后的残差
    return raw_residuals * weights

# ==========================================
# 4. 设置仿真参数与运行环境
# ==========================================
# 假设的系统物理参数
p_vals = (
    1.0,  # m_rod
    0.5,  # m_w
    2.0,  # m_p
    0.1,  # R
    0.5,  # h
    0.05, # I
    9.81, # g
    10.0  # k (控制增益)
)

# 初始状态 [x1, x2, x3, x4] 和 初始误差 [e1, e2, e3, e4]
current_state = np.array([0.1, 0.0, 0.1, 0.0]) 
current_error = np.array([-0.1, 0.0, 0.2, 0.0])

# ==========================================
# 5. 求解控制输入 F
# ==========================================
print("\nSolving for Control Input F...")

# 优化/求解目标：寻找 F 使得 control_equations 尽可能接近 0
# 初始猜测 F = 0.0
F_guess = np.array([0.0])

# 考虑到方程可能超定(4个方程1个变量)，可以使用 least_squares，这里用 root 作为演示
from scipy.optimize import least_squares

sol = least_squares(cost_function_for_F, F_guess, args=(current_state, current_error, p_vals))

if sol.success:
    optimal_F = sol.x[0]
    print(f"Optimization Successful! Optimal F = {optimal_F:.4f}")
    
    # 验证残差
    residuals = cost_function_for_F([optimal_F], current_state, current_error, p_vals)
    print(f"Residuals for the 4 equations: {residuals}")
else:
    print("Optimization failed to converge.")