import numpy as np

def IplatModel(t, z, par, controller):
    """
    Latitude Nonlinear Model (Appell Formulation)
    
    State:
    z = [theta, theta_dot, r, r_dot]
    
    Input:
    F = General Force
    """
    # Ensure z is a flat 1D numpy array
    z = np.asarray(z, dtype=float).flatten()
    if z.size != 4:
        raise ValueError("State vector z must have four elements: [theta, theta_dot, r, r_dot]")
        
    # Controller force
    F = controller(t, z, par)
    if not np.isscalar(F) or not np.isfinite(F):
        raise ValueError("Controller output F must be a finite scalar.")
        
    # States
    theta     = z[0]
    theta_dot = z[1]
    r         = z[2]
    r_dot     = z[3]
    
    # Parameters (Mapped identically to the MPC implementation)
    R     = par.R
    g     = par.g
    m_w   = par.m_W
    m_rod = 2 * par.m_L
    m_b   = par.m_B
    h     = par.h
    
    # Appell quasi-velocities
    u1 = theta_dot
    u2 = r_dot - R * theta_dot
    
    # Appell Mass Matrix
    M_matrix = np.array([
        [m_w * R**2 + m_rod * r**2 + m_b * (R + h)**2 + par.I_w + par.I_rod + par.I_b, 0.0],
        [0.0, m_rod]
    ])
    
    # Appell Right Side (Forces/Coriolis/Gravity)
    M_rightside = np.array([
        F * R - m_rod * g * r * np.cos(theta) + m_w * g * R * np.sin(theta) + m_b * g * (R + h) * np.sin(theta) - m_rod * r * (2 * u2 * u1 + R * u1**2),
        F - m_rod * g * np.sin(theta) + m_rod * r * u1**2
    ])
    
    if not np.all(np.isfinite(M_matrix)):
        raise ValueError("M_matrix contains NaN or Inf")
        
    # Singular matrix check using condition number
    # (rcond equivalent check via reciprocal of 2-norm condition number)
    cond = np.linalg.cond(M_matrix)
    if np.isinf(cond) or (1.0 / cond) < 1e-12:
        raise RuntimeError("LatModelAppell: M_matrix is singular or poorly conditioned.")
        
    # Solve for quasi-accelerations [u1_dot, u2_dot]
    accel_u = np.linalg.solve(M_matrix, M_rightside)
    u1_dot = accel_u[0]
    u2_dot = accel_u[1]
    
    # Convert quasi-accelerations back to standard state accelerations
    # Since u1 = theta_dot           => u1_dot = theta_ddot
    # Since u2 = r_dot - R*theta_dot => u2_dot = r_ddot - R*theta_ddot
    theta_ddot = u1_dot
    r_ddot     = u2_dot + R * theta_ddot
    
    # State derivative
    dz = np.array([
        theta_dot,
        theta_ddot,
        r_dot,
        r_ddot
    ])
    
    return dz