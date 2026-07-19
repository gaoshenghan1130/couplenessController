import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from Iplatmodel import IplatModel
from Iplatparam import SystemParameters
from simu import rk4_fixed_step
from Pd import pd_controller
from L1 import L1_cp_controller
from L2 import L2_cp_controller
from L3 import L3_cp_controller

if __name__ == "__main__":
    # Initialize parameters
    par = SystemParameters()
    
    # Initial states: [theta, theta_dot, r, r_dot]
    z0 = [0.95 * np.pi/180, 0.0, 0.0, 0.0] 
    
    # Simulation time configuration
    t_start = 0.0
    t_end = 12.0
    t_span = (t_start, t_end)
    t_eval = np.linspace(t_start, t_end, 1000)
    
    controller =pd_controller  
    ode_fun = lambda t, z: IplatModel(t, z, par, controller)
    
    print("Running simulation...")
    # Solve the initial value problem
    sol = rk4_fixed_step(ode_fun, t_span, z0, dt=0.01)
    print("Simulation finished successfully!")
    
    # Extract results
    time = sol[0]
    theta = sol[1][0]
    theta_dot = sol[1][1]
    r = sol[1][2]
    r_dot = sol[1][3]

    # ==========================================
    # 4.5 RECONSTRUCT CONTROL INPUT (F)
    # ==========================================
    # Through looping over saved states, reconstruct the controller force history
    F_history = []
    for i in range(len(time)):
        z_current = [theta[i], theta_dot[i], r[i], r_dot[i]]
        F_current = controller(time[i], z_current, par)
        F_history.append(F_current)
    F_history = np.array(F_history)

    # ==========================================
    # 5. PLOTTING RESULTS
    # ==========================================
    # Changed layout to 3x2 to neatly accommodate the Control Input plot
    plt.figure(figsize=(8, 6))
    
    # Plot Theta tracking
    plt.subplot(3, 2, 1)
    plt.plot(time, theta, 'b-', label=r'$\theta$ (Actual)')
    plt.axhline(y=0, color='r', linestyle='--', label=r'$\theta$ (Ref)')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.title('Theta State Tracking')
    plt.legend()
    
    # Plot Theta dot
    plt.subplot(3, 2, 2)
    plt.plot(time, theta_dot, 'g-')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Theta Dot State')
    
    # Plot r tracking
    plt.subplot(3, 2, 3)
    plt.plot(time, r, 'b-', label=r'$r$ (Actual)')
    plt.axhline(y=0, color='r', linestyle='--', label=r'$r$ (Ref)')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('r State Tracking')
    plt.legend()
    
    # Plot r dot
    plt.subplot(3, 2, 4)
    plt.plot(time, r_dot, 'g-')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('r Dot State')
    
    # Plot Control Input F (Occupies the bottom left slot)
    plt.subplot(3, 2, 5)
    plt.plot(time, F_history, 'r-', linewidth=1.5)
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Force / Control Input (N)')
    plt.title('Control Input (F) over Time')
    
    plt.tight_layout()
    plt.show()