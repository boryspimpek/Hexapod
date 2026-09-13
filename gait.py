import math
import numpy as np
from config import l_coxa, l_femur, l_tibia

def calc_dir(jx, jy):
    magnitude = math.sqrt(jx**2 + jy**2)
    if magnitude < 1e-6:
        # drążek w pozycji neutralnej -> brak kierunku ruchu
        return (0.0, 0.0)
    return (jx / magnitude, jy / magnitude)


def calculate_trajectory(global_phase, step_length, step_height, p_start, jx, jy):
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


