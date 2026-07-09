import numpy as np
import matplotlib.pyplot as plt
from Iplatmodel import IplatModel
from Iplatparam import SystemParameters
from L1 import L1_cp_controller


def rk4_fixed_step(fun, t_span, z0, dt):
    """
    Fixed step RK4 simulator

    dz/dt = fun(t,z)

    Returns:
        t_array
        z_array
    """

    t0, tf = t_span

    N = int((tf - t0) / dt)

    t = np.linspace(t0, tf, N+1)

    z = np.zeros((len(z0), N+1))
    z[:,0] = z0

    for i in range(N):

        ti = t[i]
        zi = z[:,i]

        k1 = fun(ti, zi)

        k2 = fun(
            ti + dt/2,
            zi + dt*k1/2
        )

        k3 = fun(
            ti + dt/2,
            zi + dt*k2/2
        )

        k4 = fun(
            ti + dt,
            zi + dt*k3
        )

        z[:,i+1] = (
            zi
            + dt/6*(k1+2*k2+2*k3+k4)
        )

    return t, z



if __name__ == "__main__":

    par = SystemParameters()


    # initial state
    z0 = np.array([
        0.95*np.pi/180,
        0,
        0,
        0
    ])


    t_start = 0
    t_end = 10

    # controller frequency
    dt = 0.01      # 100 Hz


    controller = L1_cp_controller


    ode_fun = lambda t,z: IplatModel(
        t,
        z,
        par,
        controller
    )


    print("Running fixed RK4 simulation...")


    time, Z = rk4_fixed_step(
        ode_fun,
        (t_start,t_end),
        z0,
        dt
    )


    print("Simulation finished!")


    theta = Z[0]
    theta_dot = Z[1]
    r = Z[2]
    r_dot = Z[3]


    # reconstruct F
    F_history=[]

    for i in range(len(time)):

        F_history.append(
            controller(
                time[i],
                Z[:,i],
                par
            )
        )

    F_history=np.array(F_history)



    plt.figure(figsize=(10,8))


    plt.subplot(3,2,1)
    plt.plot(time,theta)
    plt.grid()
    plt.title("theta")


    plt.subplot(3,2,2)
    plt.plot(time,theta_dot)
    plt.grid()
    plt.title("theta_dot")


    plt.subplot(3,2,3)
    plt.plot(time,r)
    plt.grid()
    plt.title("r")


    plt.subplot(3,2,4)
    plt.plot(time,r_dot)
    plt.grid()
    plt.title("r_dot")


    plt.subplot(3,2,5)
    plt.plot(time,F_history)
    plt.grid()
    plt.title("Force F")


    plt.tight_layout()
    plt.show()