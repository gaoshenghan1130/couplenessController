from scipy.optimize import minimize_scalar
from Iplatparam import SystemParameters
import numpy as np


def L1_cp_controller(t, z, par = SystemParameters()):
    """
    PD Controller for the Latitude Nonlinear Model.
    """
    # Unpack states
    theta = z[0]
    theta_dot = z[1]
    r = z[2]
    r_dot = z[3]

    # We want to use the first order of each variable for xs
    # def cost(F):
    #     # Define the cost function based on the state and control input

    #     R = par.R
    #     g = par.g
    #     m = par.m  # Assuming m_rod = 2 * par.m_L
    #     G = par.G
    #     J = par.J
    #     delta_t = par.delta_t

    #     tau_theta = (
    #         F * R
    #         # + G * np.sin(theta)
    #         # - m * g * r * np.cos(theta)
    #         # - m * r * (2 * r_dot * theta_dot + R * theta_dot**2)
    #     )
    #     F_r = F # - m * g * np.sin(theta) + m * r * theta_dot**2

    #     delta_x_total = np.vstack(
    #         [
    #           tau_theta / J * delta_t,
    #           F_r / m * delta_t, 
    #           tau_theta / J, 
    #           F_r / m
    #           ]
    #     )  # Assuming delta_t is the time step

    #     K = np.diag([
    #         par.K_1,
    #         par.K_2,
    #         par.K_3,
    #         par.K_4
    #     ])

    #     e = np.array([
    #         -theta,
    #         -r,
    #         -theta_dot,
    #         -r_dot
    #     ])

    #     dx = delta_x_total.flatten()

    #     err = dx - e

    #     return err @ K @ err
    
    # resF = minimize_scalar(cost, bounds=(-1000, 1000), method='bounded')

    R = par.R
    J = par.J
    m = par.m
    dt = par.delta_t

    e1 = -theta
    e2 = -(r - R * theta)
    e3 = -theta_dot
    e4 = -(r_dot - R * theta_dot)

    upper = par.K_1 * e1 * R * dt / J + par.K_2 * e2 * dt / m + par.K_3 * e3 * R / J + par.K_4 * e4 / m 
    lower = (par.K_1 * R**2 / J**2 + par.K_2 / m**2) * dt + (par.K_3 * R**2 / J**2 + par.K_4 / m**2)

    print("Upper:", upper)
    print("Lower:", lower)
    print("Alpha:", par.alpha)

    print(par.K_1 * R * dt/J)


    return par.alpha * upper / lower

    

        
    return par.alpha * resF.x # type: ignore
