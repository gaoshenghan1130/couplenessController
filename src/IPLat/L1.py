from scipy.optimize import minimize_scalar
from Iplatparam import SystemParameters
import numpy as np

loopnum = 0

def L1_cp_controller(t, z, par = SystemParameters()):
    # Unpack states
    theta = z[0]
    theta_dot = z[1]
    r = z[2]
    r_dot = z[3]
    
    K_1 = par.K_1 
    K_2 = par.K_2
    K_3 = par.K_3 
    K_4 = par.K_4
    delta_t = par.delta_t 
    alpha = par.alpha

    # We want to use the first order of each variable for xs
    def cost(F):
        # Define the cost function based on the state and control input

        R = par.R
        g = par.g
        m = par.m  # Assuming m_rod = 2 * par.m_L
        G = par.G
        J = par.J

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
              tau_theta / J, 
              F_r / m
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

    return resF.x # type: ignore
