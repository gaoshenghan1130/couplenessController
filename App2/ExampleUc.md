## 5. Real Implementations

As this problem is triggered by the unicycle project, we will now discuss the real implementations of controller for the actuator for the unicycle lateral 2D model

### 5.1 Model definition and control design

The equation for model(Use the Appell Model so as to ease the diagonalization of the system) is given by:

$$
\begin{bmatrix} 
m_w R^2 + m_{rod} r^2 + m_p (R + h)^2 + (I_w + I_b  + I_{rod}) & 0 \\ 
0 & m_{rod} 
\end{bmatrix}
\begin{bmatrix} 
\dot{u}_1 \\ 
\dot{u}_2 
\end{bmatrix}
= \begin{bmatrix} 
F R - m_{rod} g r \cos\theta + m_w g R \sin\theta + m_p g (R + h) \sin\theta - m_{rod} r (2 u_2 u_1 + R u_1^2) \\ 
F - m_{rod} g \sin\theta + m_{rod} r u_1^2 
\end{bmatrix}
$$

Where $u_1 = \dot{\theta}$ and $u_2 = \dot{r} - R\dot{\theta}$.

In full form, replace inertia as $I = I_{rod} + I_w + I_b$, define state vector $x = [\theta, r - R\theta, \dot{\theta} , \dot{r}- R\dot{\theta} ]^T$, 

$$
\dot{x} = 
\begin{bmatrix} 
x_3 \\ 
x_4 \\ 
\frac{- m_{rod} g r \cos x_1 + [m_w g R + m_p g (R + h)] \sin x_1 - m_{rod} r (2 x_4 x_3 + R x_3^2)}{M_{11}(x)} \\ 
-g \sin x_1 + r x_3^2
\end{bmatrix} 
+ 
\begin{bmatrix} 
0 \\ 
0 \\ 
\frac{R}{M_{11}(x)} \\ 
\frac{1}{m_{rod}} 
\end{bmatrix} F
$$

Where:

$$
M_{11}(x) = m_w R^2 + m_{rod}(x_2 + R x_1)^2 + m_p (R + h)^2 + I
$$

# 5.1.1 Coupleness matrix

With python [scripts](../src/util/ucBracketCal.py) , we can compute the coupleness matrix for the unicycle model
