import numpy as np
import matplotlib.pyplot as plt

from Iplatmodel import IplatModel
from Iplatparam import SystemParameters
from simu import rk4_fixed_step
from Pd import pd_controller
from L1 import L1_cp_controller
from L2 import L2_cp_controller
from L3 import L3_cp_controller


# ============================================================
# PLOTTING CONFIGURATION
# ============================================================

# True: large fonts and thick lines for poster use
# False: smaller fonts for normal desktop viewing
POSTER_MODE = True

# Overall scaling factor for poster elements.
# Suggested range:
#   0.8: smaller poster figure
#   1.0: default
#   1.2: large figure on poster
#   1.4: very large labels
POSTER_SCALE = 1.0

# Export settings
SAVE_FIGURE = True
OUTPUT_PDF = "simulation_results_poster.pdf"
OUTPUT_PNG = "simulation_results_poster.png"
PNG_DPI = 600


def configure_plot_style(poster_mode=True, scale=1.0):
    """
    Configure global Matplotlib parameters.

    Parameters
    ----------
    poster_mode : bool
        Use enlarged fonts and thicker lines for poster figures.
    scale : float
        Overall scaling factor for fonts, lines, and markers.
    """

    if poster_mode:
        base_settings = {
            "font.size": 16,
            "axes.titlesize": 19,
            "axes.labelsize": 17,
            "xtick.labelsize": 15,
            "ytick.labelsize": 15,
            "legend.fontsize": 14,
            "lines.linewidth": 3.0,
            "axes.linewidth": 1.8,
            "xtick.major.width": 1.6,
            "ytick.major.width": 1.6,
            "xtick.major.size": 6,
            "ytick.major.size": 6,
            "grid.linewidth": 1.0,
        }
    else:
        base_settings = {
            "font.size": 11,
            "axes.titlesize": 13,
            "axes.labelsize": 12,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "legend.fontsize": 10,
            "lines.linewidth": 1.8,
            "axes.linewidth": 1.0,
            "xtick.major.width": 1.0,
            "ytick.major.width": 1.0,
            "xtick.major.size": 4,
            "ytick.major.size": 4,
            "grid.linewidth": 0.7,
        }

    scaled_settings = {}

    for key, value in base_settings.items():
        if isinstance(value, (int, float)):
            scaled_settings[key] = value * scale
        else:
            scaled_settings[key] = value

    scaled_settings.update({
        "font.family": "sans-serif",
        "mathtext.fontset": "dejavusans",
        "axes.titleweight": "bold",
        "axes.labelweight": "normal",
        "legend.frameon": True,
        "legend.framealpha": 0.9,
        "figure.dpi": 120,
        "savefig.dpi": PNG_DPI,
    })

    plt.rcParams.update(scaled_settings)


def style_axis(ax):
    """
    Apply consistent axis formatting.
    """
    ax.grid(True, alpha=0.35)
    ax.tick_params(
        axis="both",
        which="major",
        direction="out"
    )

    for spine in ax.spines.values():
        spine.set_linewidth(
            1.8 * POSTER_SCALE if POSTER_MODE else 1.0
        )


if __name__ == "__main__":

    # ========================================================
    # 1. INITIALIZE MODEL PARAMETERS
    # ========================================================

    par = SystemParameters()

    # Initial states:
    # z = [theta, theta_dot, r, r_dot]
    z0 = [
        0.95 * np.pi / 180,
        0.0,
        0.0,
        0.0
    ]

    # ========================================================
    # 2. SIMULATION CONFIGURATION
    # ========================================================

    t_start = 0.0
    t_end = 12.0
    t_span = (t_start, t_end)

    simulation_dt = 0.01

    # Select controller here:
    controller = L2_cp_controller

    # Other available controllers:
    # controller = L1_cp_controller
    # controller = L2_cp_controller
    # controller = L3_cp_controller

    ode_fun = lambda t, z: IplatModel(
        t,
        z,
        par,
        controller
    )

    # ========================================================
    # 3. RUN SIMULATION
    # ========================================================

    print("Running simulation...")

    sol = rk4_fixed_step(
        ode_fun,
        t_span,
        z0,
        dt=simulation_dt
    )

    print("Simulation finished successfully!")

    # ========================================================
    # 4. EXTRACT SIMULATION RESULTS
    # ========================================================

    time = np.asarray(sol[0])
    states = np.asarray(sol[1])

    theta = states[0]
    theta_dot = states[1]
    r = states[2]
    r_dot = states[3]

    # ========================================================
    # 5. RECONSTRUCT CONTROL INPUT F
    # ========================================================

    F_history = np.zeros_like(time, dtype=float)

    for i in range(len(time)):
        z_current = np.array([
            theta[i],
            theta_dot[i],
            r[i],
            r_dot[i]
        ])

        F_history[i] = controller(
            time[i],
            z_current,
            par
        )

    # ========================================================
    # 6. CONFIGURE PLOT STYLE
    # ========================================================

    configure_plot_style(
        poster_mode=POSTER_MODE,
        scale=POSTER_SCALE
    )

    if POSTER_MODE:
        figure_size = (
            10 * POSTER_SCALE,
            10 * POSTER_SCALE
        )
    else:
        figure_size = (10, 7.5)

    fig, axes = plt.subplots(
        3,
        2,
        figsize=figure_size
    )

    ax_theta = axes[0, 0]
    ax_theta_dot = axes[0, 1]
    ax_r = axes[1, 0]
    ax_r_dot = axes[1, 1]
    ax_force = axes[2, 0]
    ax_empty = axes[2, 1]

    # ========================================================
    # 7. PLOT THETA
    # ========================================================

    ax_theta.plot(
        time,
        theta,
        label=r"$\theta$",
        linewidth=3.2 * POSTER_SCALE
        if POSTER_MODE else 1.8
    )

    ax_theta.axhline(
        y=0.0,
        linestyle="--",
        linewidth=2.2 * POSTER_SCALE
        if POSTER_MODE else 1.2,
        label="Reference"
    )

    ax_theta.set_xlabel("Time (s)")
    ax_theta.set_ylabel(r"$\theta$ (rad)")
    ax_theta.set_title("Lean Angle")
    ax_theta.legend()
    style_axis(ax_theta)

    # ========================================================
    # 8. PLOT THETA DOT
    # ========================================================

    ax_theta_dot.plot(
        time,
        theta_dot,
        linewidth=3.2 * POSTER_SCALE
        if POSTER_MODE else 1.8
    )

    ax_theta_dot.axhline(
        y=0.0,
        linestyle="--",
        linewidth=1.8 * POSTER_SCALE
        if POSTER_MODE else 1.0
    )

    ax_theta_dot.set_xlabel("Time (s)")
    ax_theta_dot.set_ylabel(
        r"$\dot{\theta}$ (rad/s)"
    )
    ax_theta_dot.set_title("Lean Angular Velocity")
    style_axis(ax_theta_dot)

    # ========================================================
    # 9. PLOT R
    # ========================================================

    ax_r.plot(
        time,
        r,
        label=r"$r$",
        linewidth=3.2 * POSTER_SCALE
        if POSTER_MODE else 1.8
    )

    ax_r.axhline(
        y=0.0,
        linestyle="--",
        linewidth=2.2 * POSTER_SCALE
        if POSTER_MODE else 1.2,
        label="Reference"
    )

    ax_r.set_xlabel("Time (s)")
    ax_r.set_ylabel(r"$r$ (m)")
    ax_r.set_title("Rod Position")
    ax_r.legend()
    style_axis(ax_r)

    # ========================================================
    # 10. PLOT R DOT
    # ========================================================

    ax_r_dot.plot(
        time,
        r_dot,
        linewidth=3.2 * POSTER_SCALE
        if POSTER_MODE else 1.8
    )

    ax_r_dot.axhline(
        y=0.0,
        linestyle="--",
        linewidth=1.8 * POSTER_SCALE
        if POSTER_MODE else 1.0
    )

    ax_r_dot.set_xlabel("Time (s)")
    ax_r_dot.set_ylabel(
        r"$\dot{r}$ (m/s)"
    )
    ax_r_dot.set_title("Rod Velocity")
    style_axis(ax_r_dot)

    # ========================================================
    # 11. PLOT CONTROL INPUT
    # ========================================================

    ax_force.plot(
        time,
        F_history,
        linewidth=3.2 * POSTER_SCALE
        if POSTER_MODE else 1.8
    )

    ax_force.axhline(
        y=0.0,
        linestyle="--",
        linewidth=1.8 * POSTER_SCALE
        if POSTER_MODE else 1.0
    )

    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel(r"$F$ (N)")
    ax_force.set_title("Control Input")
    style_axis(ax_force)

    # The last subplot is intentionally unused.
    ax_empty.axis("off")

    # ========================================================
    # 12. FINAL LAYOUT AND EXPORT
    # ========================================================

    fig.tight_layout(
        pad=1.5 * POSTER_SCALE,
        h_pad=1.7 * POSTER_SCALE,
        w_pad=1.5 * POSTER_SCALE
    )

    if SAVE_FIGURE:
        fig.savefig(
            OUTPUT_PDF,
            bbox_inches="tight"
        )

        fig.savefig(
            OUTPUT_PNG,
            dpi=PNG_DPI,
            bbox_inches="tight"
        )

        print(f"Saved PDF figure to: {OUTPUT_PDF}")
        print(f"Saved PNG figure to: {OUTPUT_PNG}")

    plt.show()