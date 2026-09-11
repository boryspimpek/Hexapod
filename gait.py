import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ik import calculate_leg_ik
from config import l_coxa, l_femur, l_tibia
from joystick import PS4Controller

step_length = 40.0
step_height = 20.0
p_start = (0.0, 110.0, -70.0)

NUM_SAMPLES = 30


def calc_dir(jx, jy):
    magnitude = math.sqrt(jx**2 + jy**2)
    if magnitude < 1e-6:
        # drążek w pozycji neutralnej -> brak kierunku ruchu
        return (0.0, 0.0)
    return (jx / magnitude, jy / magnitude)


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
    """Liczy pozycje i kąty IK dla całego cyklu kroku, dla BIEŻĄCYCH jx, jy."""
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
        "jx": jx, "jy": jy,
    }


def setup_figure():
    fig = plt.figure(figsize=(14, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax2d = fig.add_subplot(1, 2, 2)
    return fig, ax3d, ax2d


def draw_frame(ax3d, ax2d, data):
    ax3d.cla()
    ax2d.cla()

    swing_mask = data["phase"] <= data["swing_ratio"]
    stance_mask = ~swing_mask

    ax3d.plot(data["x"][swing_mask], data["y"][swing_mask], data["z"][swing_mask],
               "o-", color="#1f77b4", linewidth=2.5, markersize=5, label="Faza swing (przenoszenie)")
    ax3d.plot(data["x"][stance_mask], data["y"][stance_mask], data["z"][stance_mask],
               "o-", color="#888888", linewidth=2.5, markersize=5, label="Faza stance (podparcie)")

    ax3d.scatter(*data["p_start"], color="green", s=120, marker="^", label="Start")
    ax3d.scatter(*data["p_end"], color="red", s=120, marker="v", label="Koniec swing")

    ax3d.set_title(
        f"Trajektoria stopy — jx={data['jx']:.2f}, jy={data['jy']:.2f}",
        fontsize=12, fontweight="bold"
    )
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.legend(loc="upper left", fontsize=8)

    ax2d.plot(data["phase"], data["coxa"], "o-", color="black", label="Coxa")
    ax2d.plot(data["phase"], data["femur"], "o-", color="orange", label="Femur")
    ax2d.plot(data["phase"], data["tibia"], "o-", color="green", label="Tibia")
    ax2d.axvline(data["swing_ratio"], color="red", linestyle="--", alpha=0.5,
                  label=f"Granica swing/stance ({data['swing_ratio']:.2f})")

    ax2d.set_title("Kąty stawów w cyklu kroku", fontsize=12, fontweight="bold")
    ax2d.set_xlabel("Faza cyklu [0-1]")
    ax2d.set_ylabel("Kąt [°]")
    ax2d.grid(True, alpha=0.3)
    ax2d.legend(loc="best", fontsize=9)


def main():
    controller = PS4Controller()
    fig, ax3d, ax2d = setup_figure()

    def update(_frame):
        stick_x, stick_y = controller.get_left_stick()

        # Mapowanie z układu pada na układ robota:
        #   pad:   stick_x = lewo/prawo, stick_y = przód/tył
        #   robot: jx = przód/tył (X), jy = boki (Y)   [konwencja jak ROS REP-103]
        # Znak przy stick_x zależy od tego, którą stronę uznajesz za "+Y":
        #   REP-103 (Y w lewo dodatnie)      -> jy = -stick_x
        #   "prawo = dodatnie" (częste w hobby) -> jy =  stick_x
        # Wybierz jedną wersję i bądź konsekwentny w całym projekcie.
        jx = stick_y
        jy = stick_x  # zakładam REP-103: Y w lewo jest dodatnie

        data = collect_trajectory_data(NUM_SAMPLES, step_length, step_height, p_start, jx, jy)
        draw_frame(ax3d, ax2d, data)

    # interval w ms -> co ile odświeżamy odczyt pada i przerysowujemy wykres
    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        controller.close()


if __name__ == "__main__":
    main()