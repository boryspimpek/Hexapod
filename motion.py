"""Shared robot frame calculations; no controller or hardware side effects."""
import config as cfg
from gait import calculate_trajectory, correct_angle
from ik import inverse_kinematics

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


def advance_motion(phase, ramp, running, elapsed):
    ramp_step = elapsed / cfg.ramp_time
    ramp = min(1.0, ramp + ramp_step) if running else max(0.0, ramp - ramp_step)
    return ramp


def next_phase(phase, ramp, elapsed):
    return (phase + cfg.gait_speed * elapsed) % 1.0 if ramp > 0.0 else 0.0
