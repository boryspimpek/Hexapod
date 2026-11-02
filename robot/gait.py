"""Kierunek ruchu i trajektoria stopy."""
import math
from . import config as cfg


def gait_settings(name=None):
    name = cfg.DEFAULT_GAIT if name is None else name
    if not isinstance(name, str) or name not in cfg.GAITS:
        raise ValueError(f"Nieznany gait: {name!r}")
    settings = cfg.GAITS[name]
    ratio, offsets = settings["swing_ratio"], settings["phase_offsets"]
    if not math.isfinite(ratio) or not 0 < ratio < 1:
        raise ValueError("swing_ratio musi być pomiędzy 0 a 1")
    if set(offsets) != set(cfg.LEGS) or any(
            not math.isfinite(v) or not 0 <= v < 1 for v in offsets.values()):
        raise ValueError("phase_offsets: wymagane fazy [0, 1) dla wszystkich nóg")
    return settings


def next_gait(current):
    for name in cfg.GAIT_CYCLE:
        gait_settings(name)
    if not cfg.GAIT_CYCLE:
        raise ValueError("GAIT_CYCLE nie może być pusty")
    if current not in cfg.GAIT_CYCLE:
        return cfg.GAIT_CYCLE[0]
    return cfg.GAIT_CYCLE[(cfg.GAIT_CYCLE.index(current) + 1) % len(cfg.GAIT_CYCLE)]


def stride_progress(phase, swing_ratio):
    """0 -> 1 podczas przenoszenia nogi; 1 -> 0 podczas podparcia."""
    if phase < swing_ratio:
        return phase / swing_ratio
    return 1 - (phase - swing_ratio) / (1 - swing_ratio)

def calc_dir(jx, jy):
    magnitude = math.hypot(jx, jy)
    if magnitude < 1e-6:
        return (0.0, 0.0)
    return (jx / magnitude, jy / magnitude)


def calculate_trajectory(global_phase, cur_step_length, cur_step_height, p_start, jx, jy,
                         *, swing_ratio=None):
    if swing_ratio is None:
        swing_ratio = gait_settings()["swing_ratio"]
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
