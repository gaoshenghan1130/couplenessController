from scipy.optimize import minimize_scalar
from Iplatparam import SystemParameters
import numpy as np

loopnum = 0

def L3_cp_controller(t, z, par=SystemParameters()):
    # Unpack states
    theta = z[0]
    theta_dot = z[1]
    r = z[2]
    r_dot = z[3]

    # Same gain scaling as your L1 version
    K_1 = par.K_1
    K_2 = par.K_2 
    K_3 = par.K_3
    K_4 = par.K_4

    delta_t = par.delta_t
    alpha = par.alpha

    # L3 high-order strength.
    # Start small, because cubic terms are very aggressive.
    beta_3 = getattr(par, "beta_3", 1.00)

    def cost(F):
        R = par.R
        g = par.g
        m = par.m
        G = par.G
        J = par.J

        # If Gamma_34 / Gamma_43 are not defined in par,
        # set them to 0 first so the controller can run.
        Gamma_34 = getattr(par, "Gamma_34", 0.0)
        Gamma_43 = getattr(par, "Gamma_43", 0.0)

        # ---------------------------------------------------------
        # Base generalized torque / force
        # ---------------------------------------------------------
        # Version matching your current L1:
        tau_theta = (
            F * R
            + G * np.sin(theta)
            - m * g * r * np.cos(theta)
            - m * r * (2 * r_dot * theta_dot - R * theta_dot**2)
        )

        F_r = (
            F
            - m * g * np.sin(theta)
            + m * r * theta_dot**2
        )

        # ---------------------------------------------------------
        # L1 terms
        # ---------------------------------------------------------
        dx1_L1 = tau_theta / J * delta_t
        dx2_L1 = F_r / m * delta_t
        dx3_L1 = tau_theta / J
        dx4_L1 = F_r / m

        # ---------------------------------------------------------
        # L3 correction terms from the coupleness expansion
        # ---------------------------------------------------------
        # Be careful: these contain tau^3 and F_r^3, so they can explode.
        tau_cubed = tau_theta**3
        Fr_cubed = F_r**3

        dx1_L3 = (
            - r * (J * theta_dot + m * R * r_dot) / (3 * J**2)
            * (tau_cubed / J**3 + Fr_cubed / m**3)
            * delta_t**2
        )

        dx2_L3 = (
            R * r * theta_dot / (3 * J)
            * (tau_cubed / J + Fr_cubed / m**3)
            * delta_t**2
        )

        dx3_L3 = (
            - 2 * r * theta_dot * F_r / J * delta_t
            + Gamma_34 * Fr_cubed / (6 * J**3 * m**3) * delta_t**2
        )

        dx4_L3 = (
            r * theta_dot * tau_theta / (J * m) * delta_t
            + Gamma_43 * tau_cubed / (6 * J**5 * m) * delta_t**2
        )

        delta_x_total = np.array([
            dx1_L1 + beta_3 * dx1_L3,
            dx2_L1 + beta_3 * dx2_L3,
            dx3_L1 + beta_3 * dx3_L3,
            dx4_L1 + beta_3 * dx4_L3
        ])

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

        err = delta_x_total - e

        return err @ K @ err

    bound = 10000

    resF = minimize_scalar(cost, bounds=(-bound, bound), method="bounded")

    return resF.x  # type: ignore