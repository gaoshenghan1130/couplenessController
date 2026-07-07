import sympy as sp
from sympy import latex
import pickle
import os

# ==============================================================================
# 1. Define Symbolic Variables
# ==============================================================================
x1, x2, x3, x4 = sp.symbols('x1 x2 x3 x4', real=True)
m_rod, m_w, m_p, R, h, I, g, F = sp.symbols('m_rod m_w m_p R h I g F', positive=True, real=True)
dt, k = sp.symbols('dt k', positive=True, real=True)

dx1, dx2, dx3, dx4 = sp.symbols('dx1 dx2 dx3 dx4', real=True)
e1, e2, e3, e4 = sp.symbols('e1 e2 e3 e4', real=True)

X = sp.Matrix([x1, x2, x3, x4])
dX = sp.Matrix([dx1, dx2, dx3, dx4])
E_err = sp.Matrix([e1, e2, e3, e4])

r_expr = x2 + R * x1

# ==============================================================================
# 2. Define System Nonlinear Dynamics
# ==============================================================================
M11 = m_w * R**2 + m_rod * (r_expr)**2 + m_p * (R + h)**2 + I

g0_3_num = -m_rod * g * r_expr * sp.cos(x1) + (m_w * g * R + m_p * g * (R + h)) * sp.sin(x1) - m_rod * r_expr * (2 * x4 * x3 + R * x3**2)
g0_3 = g0_3_num / M11
g0_4 = -g * sp.sin(x1) + r_expr * x3**2

g0 = sp.Matrix([x3, x4, g0_3, g0_4])
g1 = sp.Matrix([0, 0, R / M11, 1 / m_rod])

f = g0 + g1 * F

# ==============================================================================
# 3. Compute Lie Brackets
# ==============================================================================
def lie_bracket(A, B, x_vec):
    Jac_A = A.jacobian(x_vec)
    Jac_B = B.jacobian(x_vec)
    return Jac_B * A - Jac_A * B

Jac_f = f.jacobian(X)
ad_g0_g1 = lie_bracket(g0, g1, X)
ad_g0_2_g1 = lie_bracket(g0, ad_g0_g1, X)

# ==============================================================================
# 4. Construct Coupleness Matrix C
# ==============================================================================
print("Constructing Coupleness Matrix C...")
C = sp.eye(4)

for i in range(4):
    for j in range(4):
        if i != j:
            inv_fi = 1 / f[i] if f[i] != 0 else sp.symbols(f'1/f_{i+1}')
            term1 = Jac_f[j, i] * inv_fi * dX[i]
            term2 = sp.Rational(1, 6) * ad_g0_2_g1[j] * (dX[i])**2 
            C[j, i] = term1 + term2

# ==============================================================================
# 5. Formulate Control Equations
# ==============================================================================
print("Formulating relationship between Control F and Error E...")

# 严格基于系统动力学状态导数进行映射组合，消除开环常数退化
dx_to_dynamics_mapping = {
    dx1: f[0] * dt,
    dx2: f[1] * dt,
    dx3: f[2] * dt,
    dx4: f[3] * dt
}

Delta_X_pred = sp.zeros(4, 1)
for i in range(4):
    # 将 C 矩阵中的对应列与动力学真值进行积分预测叠加
    Delta_X_pred += C[:, i].subs(dx_to_dynamics_mapping)

control_equations_symbolic = Delta_X_pred + k * E_err
control_equations = control_equations_symbolic.subs(dt, sp.Rational(1, 1))

print("\n--- RESULTS ---")
for idx in range(4):
    print(f"\nEquation for dimension {idx+1} (x{idx+1} error):")
    print(sp.expand(control_equations[idx]).evalf(5))

# 创建保存路径
os.makedirs("./src/util", exist_ok=True)

# ==============================================================================
# 6. Export Markdown Documentation and Pickled Equations
# ==============================================================================
with open("./src/util/uc_coupleness.md", "w", encoding="utf-8") as f_md:
    f_md.write("## Control Equations (Dynamic Coupleness Form)\n\n")
    for i in range(4):
        f_md.write("$$\n")
        f_md.write(latex(sp.expand(control_equations[i])) + r"=0")
        f_md.write("\n$$\n\n")

export_data = {
    'control_equations': control_equations,
    'f': f
}

with open("./src/util/system_equations.pkl", "wb") as f_pkl:
    pickle.dump(export_data, f_pkl)

print("\nMarkdown has been written to ./src/util/uc_coupleness.md")
print("Symbolic equations have been exported to ./src/util/system_equations.pkl")