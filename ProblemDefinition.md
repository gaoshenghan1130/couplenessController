# Why use coupleness for controlling nonlinear systems?

## Logical reasoning

For each nonlinear system, the interactions between its components can be complex and difficult to manage. I want to give a definition of coupleness that captures the essence of these interactions and use it to provide a framework for control strategies.

Assume the nonlinear system:

$$
\dot{x} = f(x, u)
$$

**Approach 1:**

Define the "Super Jacobian" matrix to be the coupleness measurement matrix $\mathcal{C}$.


$$
\mathcal{C}^z(z) = \gamma A(z) + \gamma^2 A^2(z) + \gamma^3 A^3(z) + \cdots \\
\implies \mathcal{C}^z(z) = (I - \gamma A(z))^{-1} - I
$$

$$
\mathcal{C}^u(z) = \mathcal{C}^z(z) \cdot B(z)
$$

Where $A(z) = \frac{\partial f}{\partial x}$ and $B(z) = \frac{\partial f}{\partial u}$.

This way it can capture whether certain variable will be affected in deeper layer or relative degree in the system.

**Approach 2:**

