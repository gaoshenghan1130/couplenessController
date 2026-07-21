# More rigid proof for controller design and backward propagation of the control law.

[TOC]


Previously, we have:

$$
x_i(t) = x_i(0) + t \cdot L_f x_i + \frac{t^2}{2!} L_f^2 x_i + \frac{t^3}{3!} L_f^3 x_i + \ldots \tag{Eq. A}
$$

and 

$$
\Delta x_j \approx \underbrace{\left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \Delta x_i \cdot t}_{\text{First order}} + \underbrace{\frac{1}{2} \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) \cdot (\Delta x_i)^2 \cdot t}_{\text{second order}} \\
+ \underbrace{\frac{1}{6} \left( \left. \frac{\partial^3 f_j}{\partial x_i^3} \right|_{x_0} \right) \cdot (\Delta x_i)^3 \cdot t}_{\text{third order}} + \ldots \tag{Eq. B}
$$

## Condition

Given a system with dynamics:

$$
\dot{x} = f(x,u) = g_0(x) + g_1(x) u
$$

### Eq. A

**Eq. A** is the Taylor expansion of the system dynamics, which assumes:

1. $f$ is smooth enough.
2. The time step $t$ is small enough such that the higher order terms are negligible.
3. $u$ is constant over the time step $t$.

### Eq. B

**Eq. B** is the Taylor expansion of the state $x_j$ with respect to the actuated state $x_i$, which is derived from:

Assume $x_j$ will a have a certain displacement $\Delta x_j$ when $x_i$ is not actuated, i.e., $f(x,u_j)|_{x_0}  = (\dot{x_j})_{origin}$.

When $x_i$ is actuated, define $X_i = x_i \vec{e_i}$. The change of $x_j$ will be $f(x_0 + X_i, u_j + u_i) = (\dot{x_j})_{affected}$, when $f(x_0, u_i) = \dot{x_i}$

Here if we assume the actuation on each of the dimensions are independent, i.e., $f(x_0 + X_i, u_j + u_i) - f(x_0, u_j)$ is independent of $u_k, k \neq i$. Then it can be rewriten as:

$$
f(x_0 + X_i, u_j + u_i) - f(x_0, u_j) = f(x_0 + X_i) - f(x_0) = (\dot{x_j})_{affected} - (\dot{x_j})_{origin} = \dot{x_j}
$$

And this is the actual $\Delta x_j$ we want to compute. And from this prospective:

$$
\Delta x_j = \big(f(x_0 + X_i) - f(x_0))\cdot t = (\frac{\partial f_j}{\partial x_i} \Delta x_i  + \frac{1}{2} \frac{\partial^2 f_j}{\partial x_i^2} (\Delta x_i)^2 + \frac{1}{6} \frac{\partial^3 f_j}{\partial x_i^3} (\Delta x_i)^3 + \ldots \big)\cdot t
$$ 

As a result, we can see that the condition of **Eq. B** is the independent actuation of each dimension:

$$
f(x_0 + X_i, u_j + u_i) - f(x_0, u_j)\\
= f(x_0 + X_i, \delta u_j e_j + \delta u_i e_i) - f(x_0, u_j) \\
= g_0(x_0 + X_i) - g_0(x_0) + g_1(x_0 + X_i) (\delta u_i e_i + \delta u_j e_j) - g_1(x_0) \delta u_j e_j \\
= g_0(x_0 + X_i) - g_0(x_0) + g_1(x_0 + X_i) \delta u_i e_i + g_1(x_0 + X_i) \delta u_j e_j - g_1(x_0) \delta u_j e_j \\
$$

Thus to satisfy the condition, we need to have:

$$
g_1(x_0 + X_i) \delta u_j e_j - g_1(x_0) \delta u_j e_j = 0 \\
\iff g_{1_k}(x_0 + X_i) = g_{1_k}(x_0), \forall k \neq i
$$

Or

$$
g_1(x_0 + X_i) = g_1(x_0) + \delta g_{1_i} e_i
$$

## Propagation

### Get $\Delta t$

From **Eq. A** we can calulate $\Delta t$ from the expansions:

$$
\Delta x_i = x_i(t) - x_i(0) = t \cdot L_f x_i + \frac{t^2}{2!} L_f^2 x_i + \frac{t^3}{3!} L_f^3 x_i + \ldots
$$

#### First order approximation

Straigtforwardly, we can get the first order approximation of $\Delta t$:

$$
\Delta t \approx \frac{\Delta x_i}{L_f x_i}
$$

#### Second order approximation

By Lagrange–Bürmann inversion formula ($\dot{x}_i \neq 0$):

$$
t \approx \frac{\Delta x_i}{\dot{x}_i} - \frac{\ddot{x}_i}{2 \dot{x}_i^3} (\Delta x_i)^2 + \mathcal{O}((\Delta x_i)^3)
$$

In Lie derivative notation, we have:

$$
t \approx \frac{\Delta x_i}{L_f x_i} - \frac{L_f^2 x_i}{2 (L_f x_i)^3} (\Delta x_i)^2 + \mathcal{O}((\Delta x_i)^3)
$$

### Get $\Delta t \to \Delta x_j$

As we have derived in **Eq. B**, we can get the $\Delta x_i \to \Delta x_j$ as:

$$
\Delta x_j \approx \underbrace{\left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \Delta x_i \cdot t}_{\text{First order}} + \underbrace{\frac{1}{2} \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) \cdot (\Delta x_i)^2 \cdot t}_{\text{second order}} + \underbrace{\frac{1}{6} \left( \left. \frac{\partial^3 f_j}{\partial x_i^3} \right|_{x_0} \right) \cdot (\Delta x_i)^3 \cdot t}_{\text{third order}} + \ldots
$$

#### First order approximation

Plugin both first order approximation of $\Delta t$ and $\Delta x_j$:

$$
\Delta x_j \approx \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \Delta x_i \cdot t = \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \Delta x_i \cdot \frac{\Delta x_i}{L_f x_i} = \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{(\Delta x_i)^2}{L_f x_i}
$$

Recall the formula of Lie derivative:

$$
L_f x_i = \sum_{k=1}^{n} \frac{\partial x_i}{\partial x_k} f_k(x) = f_i(x) = \dot{x}_i
$$

Plugin in the formula, we have:

$$
\Delta x_j \approx \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{(\Delta x_i)^2}{\dot{x}_i}
$$

Thus we have the first order approximation of coupleness between $x_i$ and $x_j$:

$$
\frac{\Delta x_j}{\Delta x_i} \approx \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{\Delta x_i}{\dot{x}_i}
$$

#### Second order approximation

Also plugin both second order approximation of $\Delta t$ and $\Delta x_j$:

$$
\begin{aligned}
\Delta x_j &\approx \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \Delta x_i \cdot t + \frac{1}{2} \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) \cdot (\Delta x_i)^2 \cdot t \\
&\approx \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{(\Delta x_i)^2}{\dot{x}_i} - \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{\ddot{x}_i}{2 \dot{x}_i^3} (\Delta x_i)^3 \\
&+ \frac{1}{2} \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) \cdot (\Delta x_i)^2 \cdot \left( \frac{\Delta x_i}{\dot{x}_i} - \frac{\ddot{x}_i}{2 \dot{x}_i^3} (\Delta x_i)^2 \right) \\
\end{aligned}
$$

For simplicity, we can ignore terms with orders higher than 3:

$$
\begin{aligned}
\Delta x_j &\approx \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{(\Delta x_i)^2}{\dot{x}_i} + \frac{1}{2} \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) \cdot (\Delta x_i)^3 \cdot \frac{1}{\dot{x}_i} - \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{\ddot{x}_i}{2 \dot{x}_i^3} (\Delta x_i)^3 \\
&= \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{(\Delta x_i)^2}{\dot{x}_i} + \frac{1}{2}\left(  \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) - \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{\ddot{x}_i}{\dot{x}_i^2}  \right) \frac{(\Delta x_i)^3}{\dot{x}_i}
\end{aligned}
$$

So the coupleness between $x_i$ and $x_j$ with second order approximation is:

$$
\frac{\Delta x_j}{\Delta x_i} \approx \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{\Delta x_i}{\dot{x}_i} + \frac{1}{2}\left(  \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) - \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{\ddot{x}_i}{\dot{x}_i^2}  \right) \frac{(\Delta x_i)^2}{\dot{x}_i}
$$

For the second term, notice that we can construct its original form as follows:

>Notice that:
$$
(\frac{f(x)}{g(x)})' = \frac{f'(x) g(x) - f(x) g'(x)}{g^2(x)}
$$

Notice $ \frac{d}{dx_i}\big( \frac{\frac{\partial f_j}{\partial x_i}}{\dot{x}_i} \big)= \frac{\partial^2 f_j}{\partial x_i^2} \cdot \frac{1}{\dot{x}_i} - \frac{\partial f_j}{\partial x_i} \cdot \frac{d}{dx_i} \left( \frac{1}{\dot{x}_i} \right) = \frac{\partial^2 f_j}{\partial x_i^2} \cdot \frac{1}{\dot{x}_i} - \frac{\partial f_j}{\partial x_i} \cdot \left( - \frac{1}{\dot{x}_i^2} \cdot \frac{d \dot{x}_i}{dx_i} \right) $. 

And through chain rule $\frac{d \dot{x}_i}{dx_i} = \frac{d \dot{x}_i}{dt} \cdot \frac{dt}{dx_i} = \frac{\ddot{x}_i}{\dot{x}_i}$, we have:

$$
\frac{d}{dx_i}\big( \frac{\frac{\partial f_j}{\partial x_i}}{\dot{x}_i} \big) = \frac{\partial^2 f_j}{\partial x_i^2} \cdot \frac{1}{\dot{x}_i} - \frac{\partial f_j}{\partial x_i} \cdot \frac{\ddot{x}_i}{\dot{x}_i^2} \\
= \left(  \left( \left. \frac{\partial^2 f_j}{\partial x_i^2} \right|_{x_0} \right) - \left( \left. \frac{\partial f_j}{\partial x_i} \right|_{x_0} \right) \cdot \frac{\ddot{x}_i}{\dot{x}_i^2}  \right) \frac{1}{\dot{x}_i}
$$

Define coupleness "Gain" as $G_{ij} = \frac{1}{\dot{x}_i} \frac{\partial f_j}{\partial x_i}$, we have:

$$
\frac{\Delta x_j}{\Delta x_i} \approx G_{ij} \Delta x_i + \frac{1}{2} \frac{d G_{ij}}{dx_i} (\Delta x_i)^2
$$

From this formate, we can guess that the higher order approximation of coupleness can be expressed as:

$$
\frac{\Delta x_j}{\Delta x_i} \approx G_{ij} \Delta x_i + \frac{1}{2} \frac{d G_{ij}}{dx_i} (\Delta x_i)^2 + \frac{1}{6} \frac{d^2 G_{ij}}{dx_i^2} (\Delta x_i)^3 + \ldots \\
= \sum_{k=1}^{\infty} \frac{1}{k!} \frac{d^{k-1} G_{ij}}{dx_i^{k-1}} (\Delta x_i)^k \\
= \int_{x_0}^{x_0 + \Delta x_i} G_{ij} dx_i
$$

We can then define coupleness matrix as:

$$
\mathcal{C}_{ji} = \int_{x_0}^{x_0 + \Delta x_i} G_{ij} dx_i
= \int_{x_0}^{x_0 + \Delta x_i} \frac{1}{\dot{x}_i} \frac{\partial f_j}{\partial x_i} dx_i
$$

Notice that if $\Delta x_i = \dot{x}_i \cdot t$, then all the $\dot{x}_i$ will cancel out in the intergral.

We can thus get the next step displacement as:

$$
\Delta x = \mathcal{C} u
$$
