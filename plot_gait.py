import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from config import l_coxa, l_femur, l_tibia
from ik import calculate_joints, inverse_kinematics
from gait import calculate_trajectory
from joystick import PS4Controller

NUM_SAMPLES = 30
servo_id = 10  # podaj serwo_id dla nogi, którą chcesz wizualizować (np. 4 dla prawej przedniej nogi)

# Wartości początkowe (startowe) — dalej sterowane suwakami w oknie wykresu
step_length = 80.0  # długość kroku w mm
step_height = 40.0  # wysokość unoszenia stopy w mm
p_start = (-60, 165, -100)  # pozycja startowa stopy w mm (x, y, z) w układzie współrzędnych nogi

# Zakresy suwaków — dostosuj do swojego robota, jeśli trzeba
STEP_LENGTH_MIN, STEP_LENGTH_MAX = 20.0, 150.0
STEP_HEIGHT_MIN, STEP_HEIGHT_MAX = 5.0, 80.0


def setup_figure():
    fig = plt.figure(figsize=(14, 7))
    # Zostawiamy miejsce na dole na suwaki
    fig.subplots_adjust(bottom=0.22)
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax2d = fig.add_subplot(1, 2, 2)
    return fig, ax3d, ax2d


def setup_sliders(fig):
    """Tworzy suwaki do sterowania step_length i step_height."""
    ax_step_length = fig.add_axes([0.15, 0.10, 0.3, 0.03])
    ax_step_height = fig.add_axes([0.15, 0.05, 0.3, 0.03])

    slider_step_length = Slider(
        ax_step_length,
        "Step length [mm]",
        STEP_LENGTH_MIN,
        STEP_LENGTH_MAX,
        valinit=step_length,
    )
    slider_step_height = Slider(
        ax_step_height,
        "Step height [mm]",
        STEP_HEIGHT_MIN,
        STEP_HEIGHT_MAX,
        valinit=step_height,
    )

    return slider_step_length, slider_step_height


def draw_leg_segments(ax3d, foot_point, l_coxa, l_femur, l_tibia):
    """
    Rysuje 3 odcinki nogi (coxa, femur, tibia) od (0,0,0) do foot_point,
    """
    x, y, z = foot_point
    points, angles, error_msg = calculate_joints(servo_id, x, y, z, l_coxa, l_femur, l_tibia)

    leg_xs = [p[0] for p in points]
    leg_ys = [p[1] for p in points]
    leg_zs = [p[2] for p in points]

    ax3d.plot(
        leg_xs, leg_ys, leg_zs,
        "o-",
        color="#d62728",
        linewidth=4,
        markersize=8,
        label="Noga (aktualna pozycja)",
        zorder=5,
    )

    return angles, error_msg


def draw_frame(ax3d, ax2d, data, foot_point=None):
    ax3d.cla()
    ax2d.cla()

    swing_mask = data["phase"] <= data["swing_ratio"]
    stance_mask = ~swing_mask

    # ============================================================
    # WYKRES 3D — TRAJEKTORIA STOPY
    # ============================================================

    ax3d.plot(
        data["x"][swing_mask],
        data["y"][swing_mask],
        data["z"][swing_mask],
        "o-",
        color="#1f77b4",
        linewidth=2.5,
        markersize=5,
        label="Faza swing (przenoszenie)"
    )

    ax3d.plot(
        data["x"][stance_mask],
        data["y"][stance_mask],
        data["z"][stance_mask],
        "o-",
        color="#888888",
        linewidth=2.5,
        markersize=5,
        label="Faza stance (podparcie)"
    )

    # Punkt początkowy
    ax3d.scatter(
        *data["p_start"],
        color="green",
        s=120,
        marker="^",
        label="Start"
    )

    # Koniec fazy swing
    ax3d.scatter(
        *data["p_end"],
        color="red",
        s=120,
        marker="v",
        label="Koniec swing"
    )

    # ============================================================
    # NOGA (coxa/femur/tibia) W BIEŻĄCEJ POZYCJI STOPY
    # ============================================================

    if foot_point is not None:
        angles, error_msg = draw_leg_segments(ax3d, foot_point, l_coxa, l_femur, l_tibia)
        c, f, t = angles

        info = f"Coxa={c:6.1f}°  Femur={f:6.1f}°  Tibia={t:6.1f}°"
        if error_msg:
            info += f"\n⚠ {error_msg}"

        ax3d.text2D(
            0.02, 0.02, info,
            transform=ax3d.transAxes,
            fontsize=9,
            family="monospace",
            color="darkred" if error_msg else "black",
            verticalalignment="bottom",
        )

    # ============================================================
    # STAŁE GRANICE OSI
    # ============================================================

    ax3d.set_xlim(60, -60)
    ax3d.set_ylim(0, 180)
    ax3d.set_zlim(-100, 0)

    # Stałe proporcje przestrzeni 3D
    ax3d.set_box_aspect((120, 140, 60))

    # ============================================================
    # OPIS WYKRESU 3D
    # ============================================================

    ax3d.set_title(
        f"Trajektoria stopy — jx={data['jx']:.2f}, jy={data['jy']:.2f}, "
        f"step_length={data['step_length']:.0f}mm, step_height={data['step_height']:.0f}mm",
        fontsize=11,
        fontweight="bold"
    )

    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")

    ax3d.legend(
        loc="upper left",
        fontsize=8
    )

    # ============================================================
    # WYKRES 2D — KĄTY STAWÓW
    # ============================================================

    ax2d.plot(
        data["phase"],
        data["coxa"],
        "o-",
        color="black",
        label="Coxa"
    )

    ax2d.plot(
        data["phase"],
        data["femur"],
        "o-",
        color="orange",
        label="Femur"
    )

    ax2d.plot(
        data["phase"],
        data["tibia"],
        "o-",
        color="green",
        label="Tibia"
    )

    # Granica swing / stance
    ax2d.axvline(
        data["swing_ratio"],
        color="red",
        linestyle="--",
        alpha=0.5,
        label=f"Granica swing/stance ({data['swing_ratio']:.2f})"
    )

    ax2d.set_title(
        "Kąty stawów w cyklu kroku",
        fontsize=12,
        fontweight="bold"
    )

    ax2d.set_xlabel("Faza cyklu [0-1]")
    ax2d.set_ylabel("Kąt [°]")

    ax2d.grid(
        True,
        alpha=0.3
    )

    ax2d.legend(
        loc="best",
        fontsize=9
    )


def collect_trajectory_data(num_samples, step_length, step_height, p_start, jx, jy):
    """Liczy pozycje i kąty IK dla całego cyklu kroku, dla BIEŻĄCYCH jx, jy."""
    phases, xs, ys, zs = [], [], [], []
    coxa_deg, femur_deg, tibia_deg = [], [], []
    errors = []

    for i in range(num_samples + 1):
        phase = i / float(num_samples)
        (x, y, z), (p_s, p_e) = calculate_trajectory(
            phase, step_length, step_height, p_start, jx, jy
        )

        phases.append(phase)
        xs.append(x)
        ys.append(y)
        zs.append(z)

        try:
            c, f, t = inverse_kinematics(x, y, z, l_coxa, l_femur, l_tibia)
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
        "step_length": step_length,
        "step_height": step_height,
    }


def main():
    controller = PS4Controller()
    fig, ax3d, ax2d = setup_figure()
    slider_step_length, slider_step_height = setup_sliders(fig)

    # Faza animacji nogi — porusza się cyklicznie po aktualnej trajektorii,
    # niezależnie od tego jak często zmienia się pozycja joysticka.
    anim_phase = {"value": 0.0}
    phase_step = 1.0 / NUM_SAMPLES  # o ile faza przesuwa się w każdej klatce animacji

    def update(_frame):
        stick_x, stick_y = controller.get_left_stick()

        # Mapowanie z układu pada na układ robota:
        #   pad:   stick_x = lewo/prawo, stick_y = przód/tył
        #   robot: jx = przód/tył (X), jy = boki (Y)

        jx = stick_y
        jy = stick_x

        # Bieżące wartości ze suwaków (można je zmieniać w trakcie działania)
        current_step_length = slider_step_length.val
        current_step_height = slider_step_height.val

        data = collect_trajectory_data(
            NUM_SAMPLES, current_step_length, current_step_height, p_start, jx, jy
        )

        # Bieżąca pozycja stopy — na potrzeby narysowania nogi na trajektorii
        anim_phase["value"] = (anim_phase["value"] + phase_step) % 1.0
        foot_point, _ = calculate_trajectory(
            anim_phase["value"], current_step_length, current_step_height, p_start, jx, jy
        )

        draw_frame(ax3d, ax2d, data, foot_point=foot_point)

    # interval w ms -> co ile odświeżamy odczyt pada i przerysowujemy wykres
    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)

    try:
        plt.show()
    finally:
        controller.close()


if __name__ == "__main__":
    main()