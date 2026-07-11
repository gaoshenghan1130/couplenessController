import copy
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from collections import Counter

from Iplatmodel import IplatModel
from Iplatparam import SystemParameters
from L2 import L2_cp_controller
from simu import rk4_fixed_step


# ==================================================
# Tuner Settings
# ==================================================

MAX_ITERS = 100

theta0_start_deg = 0.1
theta0_max_deg = 5.0
theta0_step_deg = 0.02

t_end = 10
dt = 0.01

# convergence criteria
theta_final_tol_deg = 0.5
r_final_tol = 0.01

# safety criteria
theta_safety_deg = 5.0
r_safety_limit = 5.0

# beta fixed
beta_fixed = 1.0

# pure random growth ranges
K_growth_range = (1.00, 1.08)
alpha_growth_range = (1.00, 1.08)
delta_t_growth_range = (1.00, 1.05)

# optional upper bounds to avoid meaningless explosion
K_abs_max = 1e8
alpha_max = 1e6
delta_t_max = 1e5


# ==================================================
# Utility
# ==================================================

def rand_growth(low, high):
    return np.random.uniform(low, high)


def safe_get_L2_params(par):
    if not hasattr(par, "K_1_L2"):
        par.K_1_L2 = par.K_1
    if not hasattr(par, "K_2_L2"):
        par.K_2_L2 = par.K_2
    if not hasattr(par, "K_3_L2"):
        par.K_3_L2 = par.K_3
    if not hasattr(par, "K_4_L2"):
        par.K_4_L2 = par.K_4

    if not hasattr(par, "alpha_L2"):
        par.alpha_L2 = par.alpha
    if not hasattr(par, "delta_t_L2"):
        par.delta_t_L2 = par.delta_t

    if not hasattr(par, "beta"):
        par.beta = beta_fixed

    return par


def grow_parameters(par):
    """
    Pure random growth:
    K_i_L2, alpha_L2, delta_t_L2 are only multiplied by random factors > 1.
    """
    new_par = copy.deepcopy(par)
    new_par = safe_get_L2_params(new_par)

    # g1 = rand_growth(*K_growth_range)
    # g2 = rand_growth(*K_growth_range)
    # g3 = rand_growth(*K_growth_range)
    # g4 = rand_growth(*K_growth_range)

    ga = rand_growth(*alpha_growth_range)
    gd = rand_growth(*delta_t_growth_range)

    # new_par.K_1_L2 *= g1
    # new_par.K_2_L2 *= g2
    # new_par.K_3_L2 *= g3
    # new_par.K_4_L2 *= g4

    new_par.alpha_L2 *= ga
    new_par.delta_t_L2 *= gd

    new_par.beta = beta_fixed

    return new_par


def parameter_valid(par):
    par = safe_get_L2_params(par)

    if abs(par.K_1_L2) > K_abs_max:
        return False
    if abs(par.K_2_L2) > K_abs_max:
        return False
    if abs(par.K_3_L2) > K_abs_max:
        return False
    if abs(par.K_4_L2) > K_abs_max:
        return False

    if abs(par.alpha_L2) > alpha_max:
        return False
    if abs(par.delta_t_L2) > delta_t_max:
        return False

    return True


def print_params(par, title="Parameters"):
    par = safe_get_L2_params(par)

    print("\n==============================")
    print(title)
    print("==============================")
    print(f"K_1_L2      : {par.K_1_L2:.10f}")
    print(f"K_2_L2      : {par.K_2_L2:.10f}")
    print(f"K_3_L2      : {par.K_3_L2:.10f}")
    print(f"K_4_L2      : {par.K_4_L2:.10f}")
    print(f"alpha_L2    : {par.alpha_L2:.10f}")
    print(f"delta_t_L2  : {par.delta_t_L2:.10f}")
    print(f"beta        : {par.beta:.10f}")


# ==================================================
# Stability Test
# ==================================================

def is_stable(par, theta0_deg, return_info=False):

    par = safe_get_L2_params(par)

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
        L2_cp_controller
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
            "reason": "stable",
            "theta0_deg": theta0_deg,
            "theta_final_deg": np.rad2deg(theta[-1]),
            "r_final": r[-1],
            "theta_max_deg": np.rad2deg(np.max(np.abs(theta))),
            "r_max": np.max(np.abs(r)),
            "theta_dot_final": theta_dot[-1],
            "r_dot_final": r_dot[-1],
        }

        # numerical failure
        if np.any(np.isnan(y)):
            info["reason"] = "nan"
            return (False, info) if return_info else False

        if np.any(np.isinf(y)):
            info["reason"] = "inf"
            return (False, info) if return_info else False

        if np.any(np.abs(y) > 1e6):
            info["reason"] = "explode"
            return (False, info) if return_info else False

        # safety
        if np.max(np.abs(theta)) > np.deg2rad(theta_safety_deg):
            info["reason"] = "theta_over"
            return (False, info) if return_info else False

        if np.max(np.abs(r)) > r_safety_limit:
            info["reason"] = "r_over"
            return (False, info) if return_info else False

        # final convergence
        theta_tail = theta[-10:]
        r_tail = r[-10:]

        theta_ok = np.max(np.abs(theta_tail)) < np.deg2rad(theta_final_tol_deg)
        r_ok = np.max(np.abs(r_tail)) < r_final_tol

        if not theta_ok:
            info["reason"] = "theta_not_converge"
            return (False, info) if return_info else False

        if not r_ok:
            info["reason"] = "r_not_converge"
            return (False, info) if return_info else False

        return (True, info) if return_info else True

    except Exception as e:
        info = {
            "reason": f"exception: {str(e)}",
            "theta0_deg": theta0_deg,
            "theta_final_deg": np.nan,
            "r_final": np.nan,
            "theta_max_deg": np.nan,
            "r_max": np.nan,
            "theta_dot_final": np.nan,
            "r_dot_final": np.nan,
        }
        return (False, info) if return_info else False


# ==================================================
# Initial Angle Continuation Tuner
# ==================================================

base_par = SystemParameters()
base_par = safe_get_L2_params(base_par)

# beta fixed at 1
base_par.beta = beta_fixed

# 如果你想从之前已经找到的 stable 参数开始，可以在这里手动覆盖：
# base_par.K_1_L2 = -3449.5264565752
# base_par.K_2_L2 = 221.5569778326
# base_par.K_3_L2 = -3134.0474624184
# base_par.K_4_L2 = 190.1283847661
# base_par.alpha_L2 = 2355.8983446066
# base_par.delta_t_L2 = 17.6127079572
# base_par.beta = beta_fixed

current_theta0_deg = theta0_start_deg

last_stable_par = None
last_stable_info = None

history = []
fail_counter = Counter()

accepted_count = 0

# First check base parameter at theta0_start_deg
base_stable, base_info = is_stable(
    base_par,
    current_theta0_deg,
    return_info=True
)

if base_stable:
    last_stable_par = copy.deepcopy(base_par)
    last_stable_info = base_info
    accepted_count += 1

    print("Base parameter is stable.")
    print(f"theta0   : {current_theta0_deg:.6f} deg")
    print(f"theta max: {base_info['theta_max_deg']:.6f} deg")
    print(f"r max    : {base_info['r_max']:.6f} m")

    current_theta0_deg = min(
        current_theta0_deg + theta0_step_deg,
        theta0_max_deg
    )
else:
    print("Base parameter is NOT stable.")
    print(f"theta0: {current_theta0_deg:.6f} deg")
    print("Reason:", base_info["reason"])


pbar = tqdm(range(MAX_ITERS), desc="L2 theta0 growth tuner")

for it in pbar:

    # use last stable parameter as seed; if none exists, use base
    seed_par = last_stable_par if last_stable_par is not None else base_par

    cand = grow_parameters(seed_par)
    cand.beta = beta_fixed

    if not parameter_valid(cand):
        fail_counter["invalid_param"] += 1
        continue

    stable, info = is_stable(
        cand,
        current_theta0_deg,
        return_info=True
    )

    reason = info["reason"] if info is not None else "none"
    fail_counter[reason] += 1

    history.append([
        it,
        1 if stable else 0,
        current_theta0_deg,
        cand.beta,
        cand.K_1_L2,
        cand.K_2_L2,
        cand.K_3_L2,
        cand.K_4_L2,
        cand.alpha_L2,
        cand.delta_t_L2,
        info["theta_final_deg"] if info else np.nan,
        info["r_final"] if info else np.nan,
        info["theta_max_deg"] if info else np.nan,
        info["r_max"] if info else np.nan,
        reason,
    ])

    if stable:
        accepted_count += 1
        last_stable_par = copy.deepcopy(cand)
        last_stable_info = info

        print("\nAccepted stable parameter:")
        print(f"iter       : {it}")
        print(f"theta0     : {current_theta0_deg:.6f} deg")
        print(f"theta max  : {info['theta_max_deg']:.6f} deg")
        print(f"r max      : {info['r_max']:.6f} m")
        print(f"theta end  : {info['theta_final_deg']:.6f} deg")
        print(f"r end      : {info['r_final']:.6f} m")

        print_params(cand, title="Stable Parameters")
        print("K1 = {:.6f}, K2 = {:.6f}, K3 = {:.6f}, K4 = {:.6f}".format(
            cand.K_1_L2, cand.K_2_L2, cand.K_3_L2, cand.K_4_L2))
        print("alpha = {:.6f}, delta_t = {:.6f}, beta = {:.6f}".format(
            cand.alpha_L2, cand.delta_t_L2, cand.beta))

        if current_theta0_deg >= theta0_max_deg:
            print("\nReached theta0_max_deg with stable parameters.")
            break

        current_theta0_deg = min(
            current_theta0_deg + theta0_step_deg,
            theta0_max_deg
        )

    pbar.set_postfix(
        theta0=f"{current_theta0_deg:.3f}°",
        init_angle=f"{current_theta0_deg:.1f}°",
        accepted=accepted_count,
        last=reason
    )


# ==================================================
# Output Final Result
# ==================================================

print("\n\n==============================")
print("Tuning Finished")
print("==============================")
print(f"Total tested cases : {len(history)}")
print(f"Accepted count     : {accepted_count}")
print(f"Last target theta0 : {current_theta0_deg:.6f} deg")

if last_stable_par is not None:
    print_params(last_stable_par, title="Last Stable Parameters")

    if last_stable_info is not None:
        print("\nLast Stable Info")
        print("------------------------------")
        print(f"theta0_deg      : {last_stable_info['theta0_deg']:.10f}")
        print(f"theta_final_deg : {last_stable_info['theta_final_deg']:.10f}")
        print(f"r_final         : {last_stable_info['r_final']:.10f}")
        print(f"theta_max_deg   : {last_stable_info['theta_max_deg']:.10f}")
        print(f"r_max           : {last_stable_info['r_max']:.10f}")
        print(f"theta_dot_final : {last_stable_info['theta_dot_final']:.10f}")
        print(f"r_dot_final     : {last_stable_info['r_dot_final']:.10f}")
else:
    print("No stable parameter found.")

print("\nFailure / Status Counts")
print("------------------------------")
for k, v in fail_counter.items():
    print(f"{k:35s}: {v}")


# ==================================================
# Plot History
# ==================================================

if len(history) > 0:
    stable_numeric_history = []
    reason_list = []

    for row in history:
        stable_numeric_history.append(row[:-1])
        reason_list.append(row[-1])

    H = np.array(stable_numeric_history, dtype=float)

    it_col = H[:, 0]
    stable_col = H[:, 1]
    theta0_col = H[:, 2]
    beta_col = H[:, 3]
    K1_col = H[:, 4]
    K2_col = H[:, 5]
    K3_col = H[:, 6]
    K4_col = H[:, 7]
    alpha_col = H[:, 8]
    delta_t_col = H[:, 9]
    theta_final_col = H[:, 10]
    r_final_col = H[:, 11]
    theta_max_col = H[:, 12]
    r_max_col = H[:, 13]

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    ax1, ax2, ax3 = axes[0]
    ax4, ax5, ax6 = axes[1]

    ax1.scatter(
        theta0_col,
        stable_col,
        c=stable_col,
        cmap="coolwarm",
        s=35,
        vmin=0,
        vmax=1
    )
    ax1.set_xlabel(r"$\theta_0$ deg")
    ax1.set_ylabel("Stable")
    ax1.set_title(r"Stable / Unstable vs Initial Angle")
    ax1.grid(True, linestyle="--", alpha=0.5)

    ax2.scatter(
        delta_t_col,
        alpha_col,
        c=stable_col,
        cmap="coolwarm",
        s=35,
        vmin=0,
        vmax=1
    )
    ax2.set_xlabel(r"$\Delta t_{L2}$")
    ax2.set_ylabel(r"$\alpha_{L2}$")
    ax2.set_title(r"$\alpha_{L2}$ vs $\Delta t_{L2}$")
    ax2.grid(True, linestyle="--", alpha=0.5)

    ax3.scatter(
        theta0_col,
        theta_max_col,
        c=stable_col,
        cmap="coolwarm",
        s=35,
        vmin=0,
        vmax=1
    )
    ax3.set_xlabel(r"$\theta_0$ deg")
    ax3.set_ylabel(r"$\theta_{max}$ deg")
    ax3.set_title(r"Peak Angle vs Initial Angle")
    ax3.grid(True, linestyle="--", alpha=0.5)

    ax4.scatter(
        theta_max_col,
        r_max_col,
        c=stable_col,
        cmap="coolwarm",
        s=35,
        vmin=0,
        vmax=1
    )
    ax4.set_xlabel(r"$\theta_{max}$ deg")
    ax4.set_ylabel(r"$r_{max}$ m")
    ax4.set_title("Transient Peak")
    ax4.grid(True, linestyle="--", alpha=0.5)

    ax5.scatter(
        K1_col,
        K3_col,
        c=stable_col,
        cmap="coolwarm",
        s=35,
        vmin=0,
        vmax=1
    )
    ax5.set_xlabel(r"$K_{1,L2}$")
    ax5.set_ylabel(r"$K_{3,L2}$")
    ax5.set_title(r"$K_{1,L2}$ vs $K_{3,L2}$")
    ax5.grid(True, linestyle="--", alpha=0.5)

    ax6.scatter(
        K2_col,
        K4_col,
        c=stable_col,
        cmap="coolwarm",
        s=35,
        vmin=0,
        vmax=1
    )
    ax6.set_xlabel(r"$K_{2,L2}$")
    ax6.set_ylabel(r"$K_{4,L2}$")
    ax6.set_title(r"$K_{2,L2}$ vs $K_{4,L2}$")
    ax6.grid(True, linestyle="--", alpha=0.5)

    plt.suptitle(
        f"L2 Controller Initial Angle Growth Tuner, initial angle = {theta0_start_deg:.2f} deg to {current_theta0_deg:.2f} deg",
        fontsize=14,
        y=1.02
    )

    plt.tight_layout()
    plt.show()