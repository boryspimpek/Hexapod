"""Kierunek ruchu i trajektoria stopy."""
import math

def calc_dir(jx, jy):
    magnitude = math.hypot(jx, jy)
    if magnitude < 1e-6:
        return (0.0, 0.0)
    return (jx / magnitude, jy / magnitude)


def calculate_trajectory(global_phase, cur_step_length, cur_step_height, p_start, jx, jy):
    swing_ratio = 0.5
    step_dir = calc_dir(jx, jy)
    half = cur_step_length / 2.0

    p_front = (p_start[0] + step_dir[0] * half, p_start[1] + step_dir[1] * half, p_start[2])
    p_back = (p_start[0] - step_dir[0] * half, p_start[1] - step_dir[1] * half, p_start[2])

    if global_phase < swing_ratio:
        t = global_phase / swing_ratio
        pos_x = p_back[0] + (p_front[0] - p_back[0]) * t
        pos_y = p_back[1] + (p_front[1] - p_back[1]) * t
        pos_z = p_start[2] + cur_step_height * math.sin(t * math.pi)
    else:
        t = (global_phase - swing_ratio) / (1.0 - swing_ratio)
        pos_x = p_front[0] + (p_back[0] - p_front[0]) * t
        pos_y = p_front[1] + (p_back[1] - p_front[1]) * t
        pos_z = p_start[2]

    return (pos_x, pos_y, pos_z)
