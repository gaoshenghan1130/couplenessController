import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

from Iplatmodel import IplatModel
from Iplatparam import SystemParameters
from L1 import L1_cp_controller
from simu import rk4_fixed_step


# ==================================================
# Monte Carlo Settings
# ==================================================

N = 300

t_end = 10
dt = 0.01

theta_min = 0.0
theta_max = 1.5      # degree
theta_tol = 0.01     # binary search accuracy


# ==================================================
# Stability Test
# ==================================================

def is_stable(par, theta0_deg):

    z0 = [
        np.deg2rad(theta0_deg),
        0.0,
        0.0,
        0.0,
    ]

    ode_fun = lambda t, z: IplatModel(
        t,
        z,
        par,
        L1_cp_controller
    )

    try:

        t, y = rk4_fixed_step(
            ode_fun,
            (0, t_end),
            z0,
            dt=dt
        )

        theta = y[0]

        if np.any(np.isnan(y)):
            return False

        if np.any(np.abs(y) > 1e6):
            return False
        
        if np.max(np.abs(theta)) > np.deg2rad(5):
            return False
        
        if np.abs(theta[-1]) > np.deg2rad(0.5):
            return False
        
        if np.abs(theta[-10]) > np.deg2rad(0.5):
            return False

        return True

    except Exception:
        return False


# ==================================================
# Binary Search
# ==================================================

def max_stable_angle(par):

    lo = theta_min
    hi = theta_max

    if not is_stable(par, lo):
        return 0.0

    while hi - lo > theta_tol:

        mid = 0.5 * (lo + hi)

        if is_stable(par, mid):
            lo = mid
        else:
            hi = mid

    return lo


# ==================================================
# Monte Carlo
# ==================================================

results = []

best_angle = 0.0
best_par = None

pbar = tqdm(range(N), desc="Monte Carlo")

for _ in pbar:

    par = SystemParameters()

    # Random sampling
    K_theta = np.random.uniform(-800, -300)
    K_r = np.random.uniform(50, 150)
    alpha = np.random.uniform(50, 100)
    delta_t = np.random.uniform(3, 10)

    par.K_1 = par.K_3 = K_theta
    par.K_2 = par.K_4 = K_r
    par.alpha = alpha
    par.delta_t = delta_t

    angle = max_stable_angle(par)

    if angle > best_angle:
        best_angle = angle
        best_par = (
            K_theta,
            K_r,
            alpha,
            delta_t
        )

    results.append([
        K_theta,
        K_r,
        alpha,
        delta_t,
        angle
    ])

    pbar.set_postfix(
        Best=f"{best_angle:.2f}°",
        Current=f"{angle:.2f}°"
    )

results = np.array(results)


# ==================================================
# Plot Two Scatter Plots
# ==================================================

# 创建 1 行 2 列的子图布局
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# 图 1：X轴为 K_r (索引1), Y轴为 K_theta (索引0)
scatter1 = ax1.scatter(
    results[:, 1],
    results[:, 0],
    c=results[:, 4],  # 用最大稳定角度着色
    cmap="turbo",
    s=30,
    vmin=theta_min,
    vmax=theta_max
)
ax1.set_xlabel(r"$K_r$")
ax1.set_ylabel(r"$K_\theta$")
ax1.set_title(r"Stability Angle by $K_r$ and $K_\theta$")
ax1.grid(True, linestyle="--", alpha=0.5)

# 图 2：X轴为 \Delta t (索引3), Y轴为 \alpha (索引2)
scatter2 = ax2.scatter(
    results[:, 3],
    results[:, 2],
    c=results[:, 4],  # 用最大稳定角度着色
    cmap="turbo",
    s=30,
    vmin=theta_min,
    vmax=theta_max
)
ax2.set_xlabel(r"$\Delta t$")
ax2.set_ylabel(r"$\alpha$")
ax2.set_title(r"Stability Angle by $\Delta t$ and $\alpha$")
ax2.grid(True, linestyle="--", alpha=0.5)

print("\n==============================")
print("Best Result")
print("==============================")
print(f"Maximum Stable Angle : {best_angle:.3f} deg")
print(f"K_theta             : {best_par[0]:.6f}")
print(f"K_r                 : {best_par[1]:.6f}")
print(f"alpha               : {best_par[2]:.6f}")
print(f"delta_t             : {best_par[3]:.6f}")

print("\nAngle Statistics")
print("----------------------------")
print("Min :", np.min(results[:,4]))
print("Max :", np.max(results[:,4]))
print("Mean:", np.mean(results[:,4]))
print("Unique:", np.unique(np.round(results[:,4],3)))


# 为两张图添加一个公用的颜色条
cbar = fig.colorbar(
    scatter2,
    ax=[ax1, ax2],
    shrink=0.85,
    pad=0.05
)
cbar.set_label("Maximum Stable Initial Angle (deg)")

# 总标题
plt.suptitle(
    "Monte Carlo Parameter Search: Selected Interaction Plots",
    fontsize=14,
    y=0.98
)

plt.show()


