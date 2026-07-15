# More rigid proof for controller design and backward propagation of the control law.


Previously, we have:

$$
x_i(t) = x_i(0) + t \cdot L_f x_i + \frac{t^2}{2!} L_f^2 x_i + \frac{t^3}{3!} L_f^3 x_i + \ldots \tag{Eq. A}
$$

and 

$$
\Delta x_j \approx \underbrace{\left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \Delta x_i \cdot t}_{\text{First order}} + \underbrace{\frac{1}{2} \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) \cdot (\Delta x_i)^2 \cdot t}_{\text{second order}} \\
+ \underbrace{\frac{1}{6} \left( \left. \frac{\partial^3 f_j}{\partial x_i^3} \right|_{x_0} \right) \cdot (\Delta x_i)^3 \cdot t}_{\text{third order}} + \ldots \tag{Eq. B}
$$

### Condition

**Eq. A** is the Taylor expansion of the system dynamics, which assumes:

1. $f$ is smooth enough.
2. The time step $t$ is small enough such that the higher order terms are negligible.
3. $u$ is constant over the time step $t$.

**Eq. B** is the Taylor expansion of the state $x_j$ with respect to the actuated state $x_i$, which is derived from:

- Assume 
