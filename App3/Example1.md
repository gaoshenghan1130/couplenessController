# Example: Route-Based Control of the Unicycle Lateral Dynamics

Given a model of the unicycle lateral dynamics:

$$
\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \\ \dot{x}_3 \\ \dot{x}_4 \end{bmatrix} = \begin{bmatrix} x_3 \\ x_4 \\ \frac{1}{J} \left[ G \sin x_1 - m g (x_2 + R x_1) \cos x_1 - m (x_2 + R x_1) (2 x_3 x_4 + R x_3^2) \right] \\ (x_2 + R x_1) x_3^2 - g \sin x_1 \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ \frac{R}{J} \\ \frac{1}{m} \end{bmatrix} F
$$

$$
g_0(x) = \begin{bmatrix} x_3 \\ x_4 \\ \frac{1}{J} \left[ G \sin x_1 - m g (x_2 + R x_1) \cos x_1 - m (x_2 + R x_1) (2 x_3 x_4 + R x_3^2) \right] \\ (x_2 + R x_1) x_3^2 - g \sin x_1 \end{bmatrix}, \quad g_1(x) = \begin{bmatrix} 0 \\ 0 \\ \frac{R}{J} \\ \frac{1}{m} \end{bmatrix}
$$

Where $x_1 = \theta$, $x_2 = r - R\theta$, $x_3 = \dot{\theta}$, $x_4 = \dot{r} - R\dot{\theta}$, and $F$ is the control input. Params defined same as [previous examples](../App2/ExampleIPlat.md).



