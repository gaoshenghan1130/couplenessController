## Example: Nonlinear Backward Propagation Through a Three-State Chain

Consider the nonlinear control-affine system

$$
\begin{aligned}
\dot{x}_1 &= u,\\
\dot{x}_2 &= x_1+\alpha x_1^2,\\
\dot{x}_3 &= x_2,
\end{aligned}
$$

where $u$ is assumed to be constant over the time interval $t\in[0,T]$. In control-affine form,

$$
\dot{x}
=
\underbrace{
\begin{bmatrix}
0\\
x_1+\alpha x_1^2\\
x_2
\end{bmatrix}
}*{g_0(x)}
+
\underbrace{
\begin{bmatrix}
1\\
0\\
0
\end{bmatrix}
}*{g_1(x)}
u.
$$

Suppose that

$$
x(0)=
\begin{bmatrix}
0&0&0
\end{bmatrix}^{\mathrm T}.
$$

Since $g_1(x)$ is constant and only its first component is nonzero, the input directly actuates only $x_1$. Therefore, the independent-actuation assumption is exactly satisfied.

### Directly actuated state

Because

$$
\dot{x}_1=u,
$$

we obtain

$$
x_1(t)=ut.
$$

Therefore,

$$
\Delta x_1=uT.
$$

### First-level propagation: $x_1\rightarrow x_2$

The second state satisfies

$$
\dot{x}_2=x_1+\alpha x_1^2
=ut+\alpha u^2t^2.
$$

Integrating over $[0,T]$ gives

$$
\begin{aligned}
\Delta x_2
&=
\int_0^T
\left(
u\tau+\alpha u^2\tau^2
\right)d\tau\
&=
\frac{1}{2}uT^2
+
\frac{\alpha}{3}u^2T^3.
\end{aligned}
$$

Using

$$
T=\frac{\Delta x_1}{u},
$$

the displacement of $x_2$ can be expressed in terms of the displacement of $x_1$:

$$
\boxed{
\Delta x_2
=
\frac{(\Delta x_1)^2}{2u}
+
\frac{\alpha(\Delta x_1)^3}{3u}
}
$$

and hence

$$
\boxed{
\frac{\Delta x_2}{\Delta x_1}
=

\frac{\Delta x_1}{2u}
+
\frac{\alpha(\Delta x_1)^2}{3u}
}.
$$

This is the exact state-to-state coupling for this example.

### Comparison with Eq. B

For the coupling from $x_1$ to $x_2$,

$$
f_2(x_1)=x_1+\alpha x_1^2,
$$

and therefore

$$
\frac{\partial f_2}{\partial x_1}
=
1+2\alpha x_1,
\qquad
\frac{\partial^2 f_2}{\partial x_1^2}
=

2\alpha.
$$

At $x_1=0$,

$$
\left.
\frac{\partial f_2}{\partial x_1}
\right|_{x_1=0}
=1,
\qquad
\left.
\frac{\partial^2 f_2}{\partial x_1^2}
\right|_{x_1=0}
=2\alpha.
$$

Directly applying Eq. B would give

$$
\Delta x_2
\approx
\left(
\Delta x_1
+
\alpha(\Delta x_1)^2
\right)T.
$$

Substituting $T=\Delta x_1/u$ gives

$$
\Delta x_2
\approx
\frac{(\Delta x_1)^2}{u}
+
\frac{\alpha(\Delta x_1)^3}{u}.
$$

This result does not agree with the exact solution. In particular, the exact coefficients are (1/2) and (1/3), rather than (1) and (1).

The discrepancy occurs because Eq. B approximates the displacement by multiplying the velocity change at the end of the interval by the entire time interval:

$$
\Delta x_j
\approx
\Delta \dot{x}_j(T),T.
$$

However, the induced velocity change grows gradually from zero to its final value. Therefore, it must be integrated over the complete trajectory.

### Correct integral expression

The induced displacement of $x_j$, relative to the unactuated trajectory, should be written as

$$
\delta x_j(T)
=
\int_0^T
\left[
f_j\left(x_0+\delta x_i(\tau)e_i\right)
-
f_j(x_0)
\right]d\tau.
$$

If $\dot{x}_i=v_i$ is constant, then

$$
\delta x_i(\tau)=v_i\tau,
\qquad
\Delta x_i=v_iT.
$$

Using the change of variable

$$
\xi=v_i\tau,
\qquad
d\tau=\frac{d\xi}{v_i},
$$

we obtain

$$
\boxed{
\delta x_j
=
\int_0^{\Delta x_i}
\frac{
f_j(x_0+\xi e_i)-f_j(x_0)
}{
v_i
}
d\xi
}
$$

Define the local differential coupling gain as

$$
G_{ji}(\xi)
=
\frac{1}{v_i}
\frac{\partial f_j}{\partial x_i}
\left(x_0+\xi e_i\right)
$$

Since

$$
f_j(x_0+\xi e_i)-f_j(x_0)
=
\int_0^\xi
\frac{\partial f_j}{\partial x_i}
\left(x_0+s e_i\right),ds,
$$

the induced displacement becomes

$$
\delta x_j
=
\int_0^{\Delta x_i}
\left(
\Delta x_i-s
\right)
G_{ji}(s),ds.
$$

Consequently, the finite-displacement coupleness is

$$
\boxed{
\mathcal C_{ji}(\Delta x_i)
=
\frac{\delta x_j}{\Delta x_i}

\frac{1}{\Delta x_i}
\int_0^{\Delta x_i}
\left(
\Delta x_i-s
\right)
G_{ji}(s),ds
}.
$$

This is a triangularly weighted integral of the local coupling gain, rather than the unweighted integral

$$
\int G_{ji},dx_i.
$$

Expanding $G_{ji}$ around $(x_i=x_{i,0})$ gives

$$
\boxed{
\mathcal C_{ji}
=
\sum_{k=1}^{\infty}
\frac{1}{(k+1)!}
\left.
\frac{d^{k-1}G_{ji}}{dx_i^{k-1}}
\right|_{x_0}
(\Delta x_i)^k
}.
$$

The first terms are therefore

$$
\boxed{
\mathcal C_{ji}
\approx
\frac{1}{2}G_{ji}\Delta x_i
+
\frac{1}{6}
\frac{dG_{ji}}{dx_i}
(\Delta x_i)^2
+
\frac{1}{24}
\frac{d^2G_{ji}}{dx_i^2}
(\Delta x_i)^3
+\cdots
}.
$$

For the current example,

$$
G_{21}(x_1)
=
\frac{1}{u}
\frac{\partial f_2}{\partial x_1}
=
\frac{1+2\alpha x_1}{u}.
$$

Thus,

$$
G_{21}(0)=\frac{1}{u},
\qquad
\left.
\frac{dG_{21}}{dx_1}
\right|_{x_1=0}
=

\frac{2\alpha}{u}.
$$

The corrected series gives

$$
\begin{aligned}
\mathcal C_{21}
&=
\frac{1}{2}
\frac{1}{u}\Delta x_1
+
\frac{1}{6}
\frac{2\alpha}{u}
(\Delta x_1)^2\
&=
\frac{\Delta x_1}{2u}
+
\frac{\alpha(\Delta x_1)^2}{3u},
\end{aligned}
$$

which agrees exactly with the previously derived analytical solution.

### Second-level backward propagation: $x_1\rightarrow x_2\rightarrow x_3$

Since

$$
\dot{x}_3=x_2,
$$

we have

$$
\begin{aligned}
\Delta x_3
&=
\int_0^T x_2(\tau),d\tau\
&=
\int_0^T
\left(
\frac{1}{2}u\tau^2
+
\frac{\alpha}{3}u^2\tau^3
\right)d\tau\
&=
\frac{1}{6}uT^3
+
\frac{\alpha}{12}u^2T^4.
\end{aligned}
$$

Substituting $T=\Delta x_1/u$ gives

$$
\boxed{
\Delta x_3
=

\frac{(\Delta x_1)^3}{6u^2}
+
\frac{\alpha(\Delta x_1)^4}{12u^2}
}.
$$

Therefore,

$$
\boxed{
\frac{\Delta x_3}{\Delta x_1}
=

\frac{(\Delta x_1)^2}{6u^2}
+
\frac{\alpha(\Delta x_1)^3}{12u^2}
}.
$$

The propagation orders can now be summarized as

$$
u
\longrightarrow
\Delta x_1=\mathcal O(T)
\longrightarrow
\Delta x_2=\mathcal O(T^2)
\longrightarrow
\Delta x_3=\mathcal O(T^3).
$$

Each additional dynamic layer introduces another time integration and therefore another factorial coefficient.

### State-coupling matrix versus input mapping

The coupleness matrix defined by

$$
\mathcal C_{ji}
=

\frac{\Delta x_j}{\Delta x_i}
$$

maps an actuated state displacement to another state displacement:

$$
\Delta x_{\mathrm{coupled}}
=

\mathcal C
\Delta x_{\mathrm{actuated}}.
$$

For a short time interval,

$$
\Delta x_{\mathrm{actuated}}
\approx
g_1(x_0)\Delta u,T.
$$

Thus, a local control-to-state approximation can be written as

$$
\boxed{
\Delta x
\approx
\mathcal C,
g_1(x_0)\Delta u,T
}.
$$

Alternatively, an input coupleness matrix may be defined as

$$
\mathcal C_u
=

\mathcal C,g_1(x_0)T,
$$

so that

$$
\Delta x
\approx
\mathcal C_u\Delta u.
$$

For the present nonlinear example, the exact control-to-state displacement is

$$
\boxed{
\Delta x(T)
=

\begin{bmatrix}
uT \\
\dfrac{1}{2}uT^2+\dfrac{\alpha}{3}u^2T^3 \\ 
\dfrac{1}{6}uT^3+\dfrac{\alpha}{12}u^2T^4 \\
\end{bmatrix}
}.
$$

This expression shows that the input-to-state map is generally nonlinear in $u$, even though the original system is control-affine.
