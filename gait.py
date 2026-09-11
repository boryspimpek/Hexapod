import math
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

    return (pos_x, pos_y, pos_z)

def main():
    num_samples = 10

    print("Faza | X (mm) | Y (mm) | Z (mm) | Coxa (deg) | Femur (deg) | Tibia (deg)")
    print("-" * 80)

    for i in range(num_samples + 1):
        phase = i / float(num_samples)

        x, y, z = calculate_hexapod_foot_trajectory(
            global_phase=phase,
            step_length=step_length,
            step_height=step_height,
            p_start=p_start,
            jx=jx,
            jy=jy
        )

        try:
            theta_coxa, theta_femur, theta_tibia = calculate_leg_ik(
                x, y, z, l_coxa, l_femur, l_tibia
            )
            angles_str = f"{theta_coxa:10.1f} | {theta_femur:11.1f} | {theta_tibia:11.1f}"
        except ValueError as e:
            angles_str = f"BŁĄD IK: {e}"

        print(f"{phase:5.2f} | {x:6.1f} | {y:6.1f} | {z:6.1f} | {angles_str}")

if __name__ == "__main__":
    main()