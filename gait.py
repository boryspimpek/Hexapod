import math
import numpy as np
import matplotlib.pyplot as plt
from ik import calculate_leg_ik
from config import l_coxa, l_femur, l_tibia

step_length = 40.0
step_height = 20.0
p_start = (0.0, 110.0, -70.0)
jx = 1
jy = 0


def calc_dir(jx, jy):
    magnitude = math.sqrt(jx**2 + jy**2)
    step_dir = (jx / magnitude, jy / magnitude)
    return step_dir


def calculate_hexapod_foot_trajectory(global_phase, step_length, step_height, p_start, jx, jy):
    swing_ratio = 0.5
    step_dir = calc_dir(jx, jy)

    p_end = (
        p_start[0] + step_dir[0] * step_length,
        p_start[1] + step_dir[1] * step_length,
        p_start[2]
    )

    if global_phase < swing_ratio:
        t = global_phase / swing_ratio
        pos_x = p_start[0] + (p_end[0] - p_start[0]) * t
        pos_y = p_start[1] + (p_end[1] - p_start[1]) * t
        pos_z = p_start[2] + step_height * math.sin(t * math.pi)
    else:
        t = (global_phase - swing_ratio) / (1.0 - swing_ratio)
        pos_x = p_end[0] + (p_start[0] - p_end[0]) * t
        pos_y = p_end[1] + (p_start[1] - p_end[1]) * t
        pos_z = p_start[2]

    return (pos_x, pos_y, pos_z), (p_start, p_end)


def collect_trajectory_data(num_samples, step_length, step_height, p_start, jx, jy):
    """Liczy pozycje i kąty IK dla całego cyklu kroku."""
    phases, xs, ys, zs = [], [], [], []
    coxa_deg, femur_deg, tibia_deg = [], [], []
    errors = []

    for i in range(num_samples + 1):
        phase = i / float(num_samples)
        (x, y, z), (p_s, p_e) = calculate_hexapod_foot_trajectory(
            phase, step_length, step_height, p_start, jx, jy
        )

        phases.append(phase)
        xs.append(x)
        ys.append(y)
        zs.append(z)

        try:
            c, f, t = calculate_leg_ik(x, y, z, l_coxa, l_femur, l_tibia)
            coxa_deg.append(c)
            femur_deg.append(f)
            tibia_deg.append(t)
            errors.append(None)
        except ValueError as e:
            coxa_deg.append(np.nan)
            femur_deg.append(np.nan)
            tibia_deg.append(np.nan)
            errors.append(str(e))

    return {
        "phase": np.array(phases),
        "x": np.array(xs), "y": np.array(ys), "z": np.array(zs),
        "coxa": np.array(coxa_deg), "femur": np.array(femur_deg), "tibia": np.array(tibia_deg),
        "errors": errors,
        "p_start": p_s, "p_end": p_e,
        "swing_ratio": 0.5,
    }


def plot_trajectory(data):
    fig = plt.figure(figsize=(14, 6))

    # --- Panel 1: trajektoria 3D stopy ---
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")

    # Kolorowanie po fazie: swing (niebieski->fiolet) vs stance (szary)
    swing_mask = data["phase"] <= data["swing_ratio"]
    stance_mask = ~swing_mask

    ax3d.plot(data["x"][swing_mask], data["y"][swing_mask], data["z"][swing_mask],
               "o-", color="#1f77b4", linewidth=2.5, markersize=5, label="Faza swing (przenoszenie)")
    ax3d.plot(data["x"][stance_mask], data["y"][stance_mask], data["z"][stance_mask],
               "o-", color="#888888", linewidth=2.5, markersize=5, label="Faza stance (podparcie)")

    ax3d.scatter(*data["p_start"], color="green", s=120, marker="^", label="Start")
    ax3d.scatter(*data["p_end"], color="red", s=120, marker="v", label="Koniec swing")

    ax3d.set_title("Trajektoria stopy w cyklu kroku", fontsize=12, fontweight="bold")
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.legend(loc="upper left", fontsize=8)

    # --- Panel 2: kąty stawów w funkcji fazy ---
    ax2d = fig.add_subplot(1, 2, 2)

    ax2d.plot(data["phase"], data["coxa"], "o-", color="black", label="Coxa")
    ax2d.plot(data["phase"], data["femur"], "o-", color="orange", label="Femur")
    ax2d.plot(data["phase"], data["tibia"], "o-", color="green", label="Tibia")

    # Zaznacz granicę swing/stance
    ax2d.axvline(data["swing_ratio"], color="red", linestyle="--", alpha=0.5,
                  label=f"Granica swing/stance ({data['swing_ratio']:.2f})")

    ax2d.set_title("Kąty stawów w cyklu kroku", fontsize=12, fontweight="bold")
    ax2d.set_xlabel("Faza cyklu [0-1]")
    ax2d.set_ylabel("Kąt [°]")
    ax2d.grid(True, alpha=0.3)
    ax2d.legend(loc="best", fontsize=9)

    # --- Adnotacja błędów IK, jeśli wystąpiły ---
    error_phases = [p for p, e in zip(data["phase"], data["errors"]) if e is not None]
    if error_phases:
        error_text = f"⚠ Błąd IK dla {len(error_phases)} próbek (fazy: " \
                     f"{', '.join(f'{p:.2f}' for p in error_phases)})"
        fig.text(0.5, 0.02, error_text, ha="center", color="darkred",
                  fontsize=10, fontweight="bold")

    plt.tight_layout(rect=[0, 0.05, 1, 1])
    plt.show()


def main():
    num_samples = 30  # więcej próbek = gładsza krzywa niż w wersji tekstowej
    data = collect_trajectory_data(num_samples, step_length, step_height, p_start, jx, jy)
    plot_trajectory(data)


if __name__ == "__main__":
    main()