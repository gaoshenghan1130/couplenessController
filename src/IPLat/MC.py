import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

from Iplatmodel import IplatModel
from Iplatparam import SystemParameters
from L3 import L3_cp_controller
from L1 import L1_cp_controller
from simu import rk4_fixed_step


# ==================================================
# Monte Carlo Settings
# ==================================================

N = 30

t_end = 10
dt = 0.01

theta0_test_deg = 0.1

# final convergence tolerance
theta_final_tol_deg = 0.5
r_final_tol = 0.01

# safety limits during simulation
theta_safety_deg = 5.0
r_safety_limit = 5.0

# choose controller
controller = L3_cp_controller
# controller = L1_cp_controller


# ==================================================
# Stability Test
# ==================================================

def is_stable(par, theta0_deg, return_info=False):

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
        controller
    )

    try:
        t, y = rk4_fixed_step(
            ode_fun,
            (0, t_end),
            z0,
            dt=dt
        )

        theta = y[0]
        theta_dot = y[1]
        r = y[2]
        r_dot = y[3]

        info = {
            "theta_final_deg": np.rad2deg(theta[-1]),
            "r_final": r[-1],
            "theta_max_deg": np.rad2deg(np.max(np.abs(theta))),
            "r_max": np.max(np.abs(r)),
            "theta_dot_final": theta_dot[-1],
            "r_dot_final": r_dot[-1],
            "reason": "stable",
        }

        # ------------------------------
        # numerical failure check
        # ------------------------------
        if np.any(np.isnan(y)):
            info["reason"] = "nan"
            return (False, info) if return_info else False

        if np.any(np.isinf(y)):
            info["reason"] = "inf"
            return (False, info) if return_info else False

        if np.any(np.abs(y) > 1e6):
            info["reason"] = "explode"
            return (False, info) if return_info else False

        # ------------------------------
        # safety check during simulation
        # ------------------------------
        if np.max(np.abs(theta)) > np.deg2rad(theta_safety_deg):
            info["reason"] = "theta_over"
            return (False, info) if return_info else False

        if np.max(np.abs(r)) > r_safety_limit:
            info["reason"] = "r_over"
            return (False, info) if return_info else False

        # ------------------------------
        # final convergence check
        # ------------------------------
        theta_tail = theta[-10:]
        r_tail = r[-10:]

        theta_final_ok = np.max(np.abs(theta_tail)) < np.deg2rad(theta_final_tol_deg)
        r_final_ok = np.max(np.abs(r_tail)) < r_final_tol

        if not theta_final_ok:
            info["reason"] = "theta_not_converge"
            return (False, info) if return_info else False

        if not r_final_ok:
            info["reason"] = "r_not_converge"
            return (False, info) if return_info else False

        return (True, info) if return_info else True

    except Exception as e:
        info = {
            "theta_final_deg": np.nan,
            "r_final": np.nan,
            "theta_max_deg": np.nan,
            "r_max": np.nan,
            "theta_dot_final": np.nan,
            "r_dot_final": np.nan,
            "reason": f"exception: {e}",
        }
        return (False, info) if return_info else False


# ==================================================
# Monte Carlo
# ==================================================

results = []

stable_count = 0

base_par = SystemParameters()

pbar = tqdm(range(N), desc=f"Monte Carlo Stable Test at {theta0_test_deg} deg")

for _ in pbar:

    par = SystemParameters()

    # --------------------------------------------------
    # Random sampling
    # --------------------------------------------------
    lambda1 = np.random.uniform(0.1, 5.0)
    lambda2 = np.random.uniform(0.1, 5.0)

    alpha = np.random.uniform(30, 150)
    delta_t = np.random.uniform(1, 10)

    beta_3 = 1.0

    # --------------------------------------------------
    # Scale original gains
    # K1 and K3 use lambda1
    # K2 and K4 use lambda2
    # --------------------------------------------------
    par.K_1 = lambda1 * base_par.K_1
    par.K_3 = lambda1 * base_par.K_3

    par.K_2 = lambda2 * base_par.K_2
    par.K_4 = lambda2 * base_par.K_4

    par.alpha = alpha
    par.delta_t = delta_t
    par.beta_3 = beta_3

    stable, info = is_stable(
        par,
        theta0_test_deg,
        return_info=True
    )

    stable_value = 1 if stable else 0
    stable_count += stable_value

    if info is None:
        theta_final_deg = np.nan
        r_final = np.nan
        theta_max_deg = np.nan
        r_max = np.nan
        theta_dot_final = np.nan
        r_dot_final = np.nan
        reason = "none"
    else:
        theta_final_deg = info["theta_final_deg"]
        r_final = info["r_final"]
        theta_max_deg = info["theta_max_deg"]
        r_max = info["r_max"]
        theta_dot_final = info["theta_dot_final"]
        r_dot_final = info["r_dot_final"]
        reason = info["reason"]

    results.append([
        lambda1,
        lambda2,
        alpha,
        delta_t,
        beta_3,
        par.K_1,
        par.K_2,
        par.K_3,
        par.K_4,
        stable_value,
        theta_final_deg,
        r_final,
        theta_max_deg,
        r_max,
        theta_dot_final,
        r_dot_final,
    ])

    pbar.set_postfix(
        Stable=f"{stable_count}/{len(results)}",
        Current="Yes" if stable else "No",
        Reason=reason
    )

results = np.array(results)


# ==================================================
# Print Summary
# ==================================================

stable_mask = results[:, 9] == 1
unstable_mask = results[:, 9] == 0

print("\n==============================")
print(f"Stable Test at theta0 = {theta0_test_deg} deg")
print("==============================")
print(f"Total samples       : {N}")
print(f"Stable samples      : {np.sum(stable_mask)}")
print(f"Unstable samples    : {np.sum(unstable_mask)}")
print(f"Stable rate         : {np.mean(stable_mask) * 100:.2f}%")

if np.any(stable_mask):
    stable_results = results[stable_mask]

    print("\nStable Parameter Ranges")
    print("----------------------------")
    print(f"lambda1 min/max     : {np.min(stable_results[:, 0]):.6f} / {np.max(stable_results[:, 0]):.6f}")
    print(f"lambda2 min/max     : {np.min(stable_results[:, 1]):.6f} / {np.max(stable_results[:, 1]):.6f}")
    print(f"alpha min/max       : {np.min(stable_results[:, 2]):.6f} / {np.max(stable_results[:, 2]):.6f}")
    print(f"delta_t min/max     : {np.min(stable_results[:, 3]):.6f} / {np.max(stable_results[:, 3]):.6f}")

    print("\nBest Stable Case by Smallest Final Error")
    print("----------------------------")

    # final error metric
    final_error = (
        np.abs(stable_results[:, 10])      # theta_final_deg
        + 100.0 * np.abs(stable_results[:, 11])  # r_final, scaled
    )

    best_idx = np.argmin(final_error)
    best = stable_results[best_idx]

    print(f"lambda1             : {best[0]:.6f}")
    print(f"lambda2             : {best[1]:.6f}")
    print(f"alpha               : {best[2]:.6f}")
    print(f"delta_t             : {best[3]:.6f}")
    print(f"beta_3              : {best[4]:.6f}")
    print(f"K_1                 : {best[5]:.6f}")
    print(f"K_2                 : {best[6]:.6f}")
    print(f"K_3                 : {best[7]:.6f}")
    print(f"K_4                 : {best[8]:.6f}")
    print(f"theta_final         : {best[10]:.6f} deg")
    print(f"r_final             : {best[11]:.6f} m")
    print(f"theta_max           : {best[12]:.6f} deg")
    print(f"r_max               : {best[13]:.6f} m")
    print(f"theta_dot_final     : {best[14]:.6f}")
    print(f"r_dot_final         : {best[15]:.6f}")


# ==================================================
# Plot Interaction Scatter Plots
# ==================================================

fig, axes = plt.subplots(1, 3, figsize=(19, 5))

ax1, ax2, ax3 = axes

# Plot 1: lambda1 vs lambda2
scatter1 = ax1.scatter(
    results[:, 0],
    results[:, 1],
    c=results[:, 9],
    cmap="coolwarm",
    s=40,
    vmin=0,
    vmax=1
)
ax1.set_xlabel(r"$\lambda_1$")
ax1.set_ylabel(r"$\lambda_2$")
ax1.set_title(r"Stable / Unstable by $\lambda_1$ and $\lambda_2$")
ax1.grid(True, linestyle="--", alpha=0.5)

# Plot 2: delta_t vs alpha
scatter2 = ax2.scatter(
    results[:, 3],
    results[:, 2],
    c=results[:, 9],
    cmap="coolwarm",
    s=40,
    vmin=0,
    vmax=1
)
ax2.set_xlabel(r"$\Delta t$")
ax2.set_ylabel(r"$\alpha$")
ax2.set_title(r"Stable / Unstable by $\Delta t$ and $\alpha$")
ax2.grid(True, linestyle="--", alpha=0.5)

# Plot 3: final r vs final theta
scatter3 = ax3.scatter(
    results[:, 10],
    results[:, 11],
    c=results[:, 9],
    cmap="coolwarm",
    s=40,
    vmin=0,
    vmax=1
)
ax3.axvline(theta_final_tol_deg, linestyle="--", alpha=0.5)
ax3.axvline(-theta_final_tol_deg, linestyle="--", alpha=0.5)
ax3.axhline(r_final_tol, linestyle="--", alpha=0.5)
ax3.axhline(-r_final_tol, linestyle="--", alpha=0.5)
ax3.set_xlabel(r"$\theta_{final}$ deg")
ax3.set_ylabel(r"$r_{final}$ m")
ax3.set_title(r"Final Convergence Check")
ax3.grid(True, linestyle="--", alpha=0.5)

cbar = fig.colorbar(
    scatter2,
    ax=axes,
    shrink=0.85,
    pad=0.03
)
cbar.set_label("Stable at 0.1 deg initial condition")

plt.suptitle(
    f"Monte Carlo Stability Test at Initial Angle = {theta0_test_deg} deg",
    fontsize=14,
    y=1.02
)

plt.show()