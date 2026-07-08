import couplenessCalculator
import sympy as sp


n = 5
m = 1

x1, x2, x3, x4, x5 = sp.symbols("x_1 x_2 x_3 x_4 x_5")
dx1, dx2, dx3, dx4, dx5 = sp.symbols(r"\Delta\ x_1 \Delta\ x_2 \Delta\ x_3 \Delta\ x_4 \Delta\ x_5")
u = sp.symbols("u")
V = sp.symbols("V")  
L1, L2 = sp.symbols("L_1 L_2")  # 两个车轮的轴距

states = [x1, x2, x3, x4, x5]
inputs = [u]
deltas = [dx1, dx2, dx3, dx4, dx5]

g0_matrix = sp.Matrix([V * sp.cos(x3), V * sp.sin(x3), 0, V/L1 * sp.sin(x3-x4), V/L2 * sp.cos(x3-x4)* sp.sin(x4-x5)])

g1_matrix = sp.Matrix([0, 0, 1, 0, 0])

mapping = {2: 0} # x3 driven by u


couplenessCalculator.generate_coupleness_analysis(
    n=n,
    m=m,
    g0=g0_matrix,
    g1=g1_matrix,
    x=states,              
    u=inputs,
    dx=deltas,
    state_to_input=mapping,
    filename="trailer_coupleness.pkl"
)