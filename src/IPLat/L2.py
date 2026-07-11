from scipy.optimize import minimize_scalar
from Iplatparam import SystemParameters
import numpy as np

init = False

def L2_cp_controller(t, z, par = SystemParameters()):
    # Unpack states
    theta = z[0]
    theta_dot = z[1]
    r = z[2]
    r_dot = z[3]


    # We want to use the first order of each variable for xs
    def cost(F):
        # Define the cost function based on the state and control input

        R = par.R
        g = par.g
        m = par.m  # Assuming m_rod = 2 * par.m_L
        G = par.G
        J = par.J

        T1 = 35
        T2 = 1
        
        K_1 = par.K_1_L2 * T1 * 1.027
        K_2 = par.K_2_L2 * T1 
        K_3 = par.K_3_L2 * T2  * 1.027 
        K_4 = par.K_4_L2 * T2 
        delta_t = par.delta_t_L2 * 0.2

        alpha = par.alpha_L2 * 0.2

        global init
        if not init:
            print(f"Using L2 controller with parameters:")
            print(f"K_1 = {K_1}, K_2 = {K_2}, K_3 = {K_3}, K_4 = {K_4}")
            print(f"delta_t = {delta_t}, alpha = {alpha}")
            init = True

        beta = 1

        tau_theta = (
            F * R
            + G * np.sin(theta)
            - m * g * r * np.cos(theta)
            - m * r * (2 * r_dot * theta_dot - R * theta_dot**2)
        )
        F_r = F - m * g * np.sin(theta) + m * r * theta_dot**2

        delta_x_total = np.vstack(
            [
              tau_theta / J * delta_t,
              F_r / m * delta_t, 
              tau_theta / J - beta * 2 * r * theta_dot * F_r/J * delta_t,
              F_r / m + beta * r * theta_dot * tau_theta / J / m * delta_t
              ]
        )  

        K = np.diag([
            K_1,
            K_2,
            K_3,
            K_4
        ])

        e = alpha * np.array([
            -theta,
            -(r - R * theta),
            -theta_dot,
            -(r_dot - R * theta_dot)
        ])

        dx = delta_x_total.flatten()

        err = dx - e


        cost = err @ K @ err

        return cost
    
    bound = 10000

    resF = minimize_scalar(cost, bounds=(-bound, bound), method='bounded')

    return  resF.x # type: ignore
