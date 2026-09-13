import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ik import calculate_joints
from gait import calculate_trajectory
from joystick import PS4Controller
from config import LEGS, LEG_ORIGINS, SERVO_ID, LEG_PHASE_OFFSET, l_coxa, l_femur, l_tibia, gait_speed, step_length, step_height, p_start

controller = PS4Controller()

LEG_COLORS = {
    leg: color for leg, color in zip(
        LEGS,
        ["#d62728", "#ff7f0e", "#2ca02c", "#1f77b4"]
    )
}


def get_leg_origin(leg):
    """Zwraca pozycję bazową (offset) danej nogi względem korpusu."""
    return LEG_ORIGINS[leg]

def draw_all_legs_frame(ax3d, all_foot_points):
    """Rysuje wszystkie nogi na jednym wykresie 3D.
    all_foot_points: dict {leg_name: (x, y, z)} - aktualna pozycja stopy każdej nogi."""
    ax3d.cla()

    for leg, foot_point in all_foot_points.items():
        ox, oy, oz = get_leg_origin(leg)
        x, y, z = foot_point

        coxa_servo_id = LEGS[leg]['coxa'][SERVO_ID]
        points, angles, error_msg = calculate_joints(coxa_servo_id, x, y, z, l_coxa, l_femur, l_tibia)

        # przesuwamy punkty nogi o origin danej nogi, żeby były rozmieszczone w przestrzeni
        leg_xs = [p[0] + ox for p in points]
        leg_ys = [p[1] + oy for p in points]
        leg_zs = [p[2] + oz for p in points]

        color = LEG_COLORS.get(leg, "#333333")

        ax3d.plot(
            leg_xs, leg_ys, leg_zs,
            "o-",
            color="darkred" if error_msg else color,
            linewidth=3,
            markersize=6,
            label=leg,
            zorder=5,
        )

    ax3d.set_xlim(150, -150)
    ax3d.set_ylim(-150, 150)
    ax3d.set_zlim(-100, 50)
    ax3d.set_box_aspect((300, 300, 150))
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.set_title("Hexapod — wszystkie nogi", fontsize=12, fontweight="bold")
    ax3d.legend(loc="upper left", fontsize=7, ncol=2)


def run_animation_all_legs():
    fig = plt.figure(figsize=(8, 8))
    ax3d = fig.add_subplot(1, 1, 1, projection="3d")

    state = {"global_phase": 0.0}
    dt = 0.02  # krok czasu animacji (50 Hz)

    def update(frame):
        stick_x, stick_y = controller.get_left_stick()
        jx, jy = stick_y, stick_x

        all_foot_points = {}
        for leg in LEGS:
            leg_phase = (state["global_phase"] + LEG_PHASE_OFFSET[leg]) % 1.0
            foot_pos, _ = calculate_trajectory(
                leg_phase, step_length, step_height, p_start, jx, jy
            )
            all_foot_points[leg] = foot_pos

        draw_all_legs_frame(ax3d, all_foot_points)

        state["global_phase"] = (state["global_phase"] + gait_speed * dt) % 1.0

    ani = FuncAnimation(fig, update, interval=dt * 1000)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_animation_all_legs()