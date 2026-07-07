### 4.2 Inverted Pendulum on a Cart

Given a state vector for the inverted pendulum on a cart system:

$$
x = [x_1, x_2, x_3, x_4]^T = [p, \theta, \dot{p}, \dot{\theta}]^T
$$

And simplify the dynamics of the system using the following equations:

$$
\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \\ \dot{x}_3 \\ \dot{x}_4 \end{bmatrix} = \underbrace{\begin{bmatrix} x_3 \\ x_4 \\ 0 \\ \frac{g \sin x_2}{L} \end{bmatrix}}_{g_0(x)} + \underbrace{\begin{bmatrix} 0 \\ 0 \\ 1 \\ -\frac{\cos x_2}{L} \end{bmatrix}}_{g_1(x)} u
$$

#### 4.2.1 Coupleness Matrix

$$
J_{g_0} = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & \frac{g \cos x_2}{L} & 0 & 0 \end{bmatrix}, \quad J_{g_1} = \begin{bmatrix} 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \\ 0 & \frac{\sin x_2}{L} & 0 & 0 \end{bmatrix}
$$

First order Lie bracket:

$$
[g_0, g_1] = \begin{bmatrix} 0 \\ 0 \\ 0 \\ \frac{x_4 \sin x_2}{L} \end{bmatrix} - \begin{bmatrix} 1 \\ -\frac{\cos x_2}{L} \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} -1 \\ \frac{\cos x_2}{L} \\ 0 \\ \frac{x_4 \sin x_2}{L} \end{bmatrix}
$$

Second order Lie bracket:

$$
[g_0, [g_0, g_1]] = J_{g_1}g_0 - J_{g_0}g_1 = \begin{bmatrix} 0 \\ -\frac{2x_4 \sin x_2}{L} \\ 0 \\ \frac{x_4^2 \cos x_2}{L} - \frac{g \cos(2x_2)}{L^2} \end{bmatrix}
$$

To make the life simpler, define $\mathcal{K} = [0, \mathcal{K}_2, 0, \mathcal{K}_4]^T$

Where:

$$
\mathcal{K}_2 = -\frac{2x_4\sin x_2}{L} - \frac{g\cos^2 x_2}{L^2}
$$

$$
\mathcal{K}_4 = \frac{x_4^2\cos x_2}{L} - \frac{g x_4\sin(2x_2)}{L^2} - \frac{g\cos(2x_2)}{L^2}
$$

And we get:

$$
\mathcal{C} = \begin{bmatrix}
1 & 0 & \frac{\Delta x_3}{\dot{x}_3} & 0 \\
\frac{1}{6}\mathcal{K}_2(\Delta x_1)^2 & 1 & \frac{1}{6}\mathcal{K}_2(\Delta x_3)^2 & \frac{\Delta x_4}{\dot{x}_4} + \frac{1}{6}\mathcal{K}_2(\Delta x_4)^2 \\
0 & 0 & 1 & 0 \\
\frac{1}{6}\mathcal{K}_4(\Delta x_1)^2 & \frac{\partial \dot{x}_4}{\partial x_2} \frac{\Delta x_2}{\dot{x}_2} + \frac{1}{6}\mathcal{K}_4(\Delta x_2)^2 & \frac{1}{6}\mathcal{K}_4(\Delta x_3)^2 & 1
\end{bmatrix}
$$

For simplicity, $\Delta x_3 = u \Delta t$ and $\Delta x_4 = -u \frac{\cos x_2}{L} \Delta t$, follow this, $\dot{x_4} = -u \frac{\cos x_2}{L}$ in $\mathcal{C}$, and we have:

$$
\mathcal{C} = \begin{bmatrix}
1 & 0 & \Delta t & 0 \\
\frac{1}{6}\mathcal{K}_2(\Delta x_1)^2 & 1 & \frac{1}{6}\mathcal{K}_2(u \Delta t)^2 & \Delta t + \frac{1}{6}\mathcal{K}_2\left( -u \frac{\cos x_2}{L} \Delta t \right)^2 \\
0 & 0 & 1 & 0 \\
\frac{1}{6}\mathcal{K}_4(\Delta x_1)^2 & \left(u \frac{\sin x_2}{L}\right) \frac{\Delta x_2}{x_4} + \frac{1}{6}\mathcal{K}_4(\Delta x_2)^2 & \frac{1}{6}\mathcal{K}_4(u \Delta t)^2 & 1
\end{bmatrix}
$$

### Controller design

$$
\Delta X_{\text{total}} = \begin{bmatrix}
1 & 0 & \Delta t & 0 \\
\frac{1}{6}\mathcal{K}_2(\Delta x_1)^2 & 1 & \frac{1}{6}\mathcal{K}_2(\Delta x_3)^2 & \Delta t + \frac{1}{6}\mathcal{K}_2(\Delta x_4)^2 \\
0 & 0 & 1 & 0 \\
\frac{1}{6}\mathcal{K}_4(\Delta x_1)^2 & \left(\frac{g\cos x_2 + u\sin x_2}{L}\right)\Delta t + \frac{1}{6}\mathcal{K}_4(\Delta x_2)^2 & \frac{1}{6}\mathcal{K}_4(\Delta x_3)^2 & 1
\end{bmatrix} \begin{bmatrix} 0 \\ 0 \\ u \\ \frac{- u\cos x_2}{L} \end{bmatrix}
$$

$$
\Delta X_{\text{total}} = \begin{bmatrix}
u\Delta t \\
\frac{1}{6}u\mathcal{K}_2(\Delta x_3)^2 - \frac{u\cos x_2}{L} \left( \Delta t + \frac{1}{6}\mathcal{K}_2(\Delta x_4)^2 \right) \\
u \\
\frac{1}{6}u\mathcal{K}_4(\Delta x_3)^2 - \frac{u\cos x_2}{L}
\end{bmatrix}
$$

We take $\Delta t = 1$ for simplicity, and $\Delta x_3 = u$, $\Delta x_4 = -u \frac{\cos x_2}{L}$, we have:

$$
\Delta X_{\text{total}} = \begin{bmatrix}
u \\
\frac{1}{6}\mathcal{K}_2(u)^3 - \frac{u\cos x_2}{L} \left( 1 + \frac{1}{6}\mathcal{K}_2(-u \frac{\cos x_2}{L})^2 \right) \\
u \\
\frac{1}{6}\mathcal{K}_4(u)^3 - \frac{u\cos x_2}{L}
\end{bmatrix}
$$

We want $\vec{e}_{error} = -\Delta X_{\text{total}}$, and we can solve for u with an optimization problem:

$$
\min_u \|\vec{e}_{error} - \Delta X_{\text{total}}\|^2 
$$

This will basically be a PD, but with third ordered terms and some sine and cosine terms. Should be better than a PD, it's just hard to tune the param.