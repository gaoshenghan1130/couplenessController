# Example of Using Coupleness-Path Identification to Control Unicycle Lateral Dynamics

## Theoretical Background

Consider a nonlinear control system

$$
\dot{x}=f(x,u).
$$

To characterize how a variation in state (x_i) affects the evolution of state (x_j), define the local differential coupleness gain as

$$
G_{ji}(x)
=
\frac{1}{\dot{x}_i}
\frac{\partial f_j}{\partial x_i},
\qquad
\dot{x}_i\neq 0.
$$

Here,

$$
\frac{\partial f_j}{\partial x_i}
$$

measures the instantaneous influence of $x_i$ on $\dot{x}_j$, while the factor $1/\dot{x}_i$ changes the parameterization from time to the displacement of $x_i$, since

$$
dx_i=\dot{x}_idt.
$$

Therefore, $G_{ji}$ can be used to identify the local propagation direction

$$
x_i\longrightarrow x_j.
$$

Suppose that $x_i$ changes by a finite amount $\Delta x_i$ over a short time interval. The displacement induced in $x_j$ is

$$
\delta x_j
=
\int_0^{\Delta x_i}
G_{ji}(\xi)
\left(
\Delta x_i-\xi
\right)
d\xi.
$$

The weighting factor

$$
\Delta x_i-\xi
$$

appears because a change in $x_i$ generated earlier in the interval affects $x_j$ for a longer remaining time.

Define the finite-displacement coupleness coefficient as

$$
\mathcal C_{ji}(\Delta x_i)
=
\frac{2}{\Delta x_i}
\int_0^{\Delta x_i}
G_{ji}(\xi)
\left(
\Delta x_i-\xi
\right)
d\xi.
$$

The induced displacement can then be written as

$$
\boxed{
\delta x_j
=
\mathcal C_{ji}(\Delta x_i)\Delta x_i
}.
$$

### Constant-Coupleness Approximation

Assume that $x_i$ changes approximately uniformly with constant $\dot{x}_i$, and that $G_{ji}$ varies negligibly over the displacement interval:

$$
G_{ji}(\xi)\approx G_{ji}(x_0).
$$

Then

$$
\begin{aligned}
\delta x_j
&\approx
G_{ji}(x_0)
\int_0^{\Delta x_i}
\left(
\Delta x_i-\xi
\right)d\xi\
&=
\frac{1}{2}
G_{ji}(x_0)
(\Delta x_i)^2.
\end{aligned}
$$

Thus,

$$
\boxed{
\delta x_j
\approx
\frac{1}{2}
G_{ji}(x_0)
(\Delta x_i)^2
}
$$

and the corresponding finite-displacement coupleness coefficient is

$$
\boxed{
\mathcal C_{ji}
\approx
G_{ji}(x_0)\Delta x_i
}.
$$

If the displacement occurs over a time interval (h), such that

$$
\Delta x_i=\dot{x}_i h,
$$

then

$$
\begin{aligned}
\delta x_j
&\approx
\frac{1}{2}
\left(
\frac{1}{\dot{x}_i}
\frac{\partial f_j}{\partial x_i}
\right)
(\dot{x}_i h)^2\
&=
\frac{h^2}{2}
\frac{\partial f_j}{\partial x_i}
\dot{x}_i.
\end{aligned}
$$

Therefore,

$$
\boxed{
\delta x_j
\approx
\frac{h^2}{2}
\frac{\partial f_j}{\partial x_i}
\dot{x}_i
}.
$$

This expression represents a one-stage state-propagation path over the interval $h$:

$$
x_i
\longrightarrow
x_j.
$$

For a control input $u$ that first induces a change in $x_i$,

$$
\delta\dot{x}_i
\approx
\frac{\partial f_i}{\partial u}\delta u,
$$

the complete propagation path becomes

$$
u
\longrightarrow
x_i
\longrightarrow
x_j.
$$

Its local finite-time contribution is associated with the product

$$
\frac{\partial f_j}{\partial x_i}
\frac{\partial f_i}{\partial u}
$$

And in complete form, the finite-time propagation path is:

$$
\delta x_i = C_{ij}\Delta x_j^{(u)} = \frac{h^2}{2}\frac{\partial f_j}{\partial x_i}B_j u_j
$$

Define $\mathcal{P}$ as the coupleness contribution of the complete propagation path, naturally:

$$
\mathcal{P}_{i \to j}^{(1)} = \frac{h^2}{2}\frac{\partial f_j}{\partial x_i}B_j
$$

Where $\mathcal{P}_{i \to j}^{(1)}$ is the first layer of coupleness contribution from $\underbrace{x_i \to x_j}_{\text{1 layer}}$.


