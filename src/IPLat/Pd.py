def pd_controller(t, z, par):
    """
    PD Controller for the Latitude Nonlinear Model.
    """
    # Unpack states
    theta     = z[0]
    theta_dot = z[1]
    r         = z[2]
    r_dot     = z[3]
    
    # Calculate tracking errors (Targets are 0)
    error_theta = - theta
    error_r     = - r

    kp_theta = -781.4476
    kd_theta = -146.7735
    kp_r = 229.6972
    kd_r = 41.4447
    
    # PD Control law
    F = (kp_theta * error_theta - kd_theta * theta_dot) + \
        (kp_r * error_r - kd_r * r_dot)
        
    return float(F)