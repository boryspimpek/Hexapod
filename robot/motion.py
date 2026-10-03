"""Shared robot frame calculations; no controller or hardware side effects."""
from . import config as cfg
import math
from .gait import calculate_trajectory, calc_dir
from .servos import correct_angle
from .kinematics import inverse_kinematics

JOINT_NAMES = ("coxa", "femur", "tibia")


def calculate_frame(phase=0.0, ramp=0.0, direction=(0.0, 0.0)):
    legs, servos = {}, {}
    for name, joints in cfg.LEGS.items():
        target = calculate_trajectory(
            (phase + cfg.LEG_PHASE_OFFSET[name]) % 1.0,
            cfg.step_length * ramp, cfg.step_height * ramp,
            cfg.p_start[name], *direction,
        )
        raw = inverse_kinematics(*target, cfg.l_coxa, cfg.l_femur, cfg.l_tibia)
        corrected = {}
        for joint, angle in zip(JOINT_NAMES, raw):
            sid = joints[joint][cfg.SERVO_ID]
            corrected[joint] = correct_angle(sid, angle)
            servos[sid] = corrected[joint]
        legs[name] = {"target": target, "raw_angles": raw, "servo_angles": corrected}
    return {"legs": legs, "servos": servos}


def advance_ramp(ramp, running, elapsed):
    ramp_step = elapsed / cfg.ramp_time
    ramp = min(1.0, ramp + ramp_step) if running else max(0.0, ramp - ramp_step)
    return ramp


def next_phase(phase, ramp, elapsed):
    return (phase + cfg.gait_speed * elapsed) % 1.0 if ramp > 0.0 else 0.0


def initial_state():
    """Nowy, niezależny stan ruchu."""
    return {"phase": 0.0, "ramp": 0.0, "direction": (0.0, 0.0)}


def step_motion(state, x, y, elapsed):
    """Zwraca (nowy stan, klatka), nie modyfikując przekazanego stanu.

    Klatka używa bieżącej fazy i nowej rampy; zwrócony stan zawiera
    fazę następnego kroku. Przy hamowaniu zachowujemy ostatni kierunek.
    """
    running = math.hypot(x, y) > cfg.stick_deadzone
    direction = calc_dir(x, y) if running else tuple(state["direction"])
    ramp = advance_ramp(state["ramp"], running, elapsed)
    frame = calculate_frame(state["phase"], ramp, direction)
    updated = {"phase": next_phase(state["phase"], ramp, elapsed),
               "ramp": ramp, "direction": direction}
    return updated, frame
