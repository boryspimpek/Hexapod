"""Shared robot frame calculations; no controller or hardware side effects."""
from . import config as cfg
import math
from .gait import calculate_trajectory, calc_dir
from .servos import correct_angle
from .kinematics import inverse_kinematics

JOINT_NAMES = ("coxa", "femur", "tibia")


def calculate_frame(phase=0.0, ramp=0.0, direction=(0.0, 0.0), *,
                    step_length=None, step_height=None, z_height=None):
    step_length = cfg.step_length if step_length is None else step_length
    step_height = cfg.step_height if step_height is None else step_height
    z_height = cfg.z_height if z_height is None else z_height
    legs, servos = {}, {}
    for name, joints in cfg.LEGS.items():
        target = calculate_trajectory(
            (phase + cfg.LEG_PHASE_OFFSET[name]) % 1.0,
            step_length * ramp, step_height * ramp,
            (*cfg.p_start[name][:2], z_height), *direction,
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


def next_phase(phase, ramp, elapsed, *, gait_speed=None):
    gait_speed = cfg.gait_speed if gait_speed is None else gait_speed
    return (phase + gait_speed * elapsed) % 1.0 if ramp > 0.0 else 0.0


def initial_state():
    """Nowy, niezależny stan ruchu."""
    return {"phase": 0.0, "ramp": 0.0, "direction": (0.0, 0.0)}


def step_motion(state, x, y, elapsed, *, gait_speed=None, step_length=None,
                step_height=None, z_height=None):
    """Zwraca (nowy stan, klatka), nie modyfikując przekazanego stanu.

    Klatka używa bieżącej fazy i nowej rampy; zwrócony stan zawiera
    fazę następnego kroku. Przy hamowaniu zachowujemy ostatni kierunek.
    """
    running = math.hypot(x, y) > cfg.stick_deadzone
    direction = calc_dir(x, y) if running else tuple(state["direction"])
    ramp = advance_ramp(state["ramp"], running, elapsed)
    frame = calculate_frame(state["phase"], ramp, direction,
                            step_length=step_length, step_height=step_height,
                            z_height=z_height)
    updated = {"phase": next_phase(state["phase"], ramp, elapsed, gait_speed=gait_speed),
               "ramp": ramp, "direction": direction}
    return updated, frame
