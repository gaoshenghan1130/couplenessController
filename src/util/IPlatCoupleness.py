import couplenessCalculator
import sympy as sp


n_dim = 4
m_dim = 2

x1, x2, x3, x4 = sp.symbols("x_1 x_2 x_3 x_4")
dx1, dx2, dx3, dx4 = sp.symbols(r"\Delta\ x_1 \Delta\ x_2 \Delta\ x_3 \Delta\ x_4")
F = sp.symbols("F")
m = sp.symbols("m")  # mass
G = sp.symbols("G")  # gravity terms
g = sp.symbols("g")  # gravity acceleration
R, J = sp.symbols("R J")  # wheel radius and moment of inertia

states = [x1, x2, x3, x4]
inputs = [F]
deltas = [dx1, dx2, dx3, dx4]

g0_matrix = sp.Matrix([ x3, 
                        x4,
                        1/J * (
                            G * sp.sin(x1) 
                            - m * g * (x2 + R * x1) * sp.cos(x1) 
                            - m * (x2 + R * x1) * (2 * x3 * x4 + R * x3**2)
                        ),
                        (x2 + R * x1) * x3**2 - g * sp.sin(x1)
                    ])

g1_matrix = sp.Matrix([0, 0, R/J, 1/m])
mapping = {2: 0, 3: 0} # x3 and x4 driven by u_1 and u_2


couplenessCalculator.generate_coupleness_analysis(
    n=n_dim,
    m=m_dim,
    g0=g0_matrix,
    g1=g1_matrix,
    x=states,              
    u=inputs,
    dx=deltas,
    state_to_input=mapping,
    filename="trailer_coupleness.pkl"
)