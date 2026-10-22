"""Shared robot frame calculations; no controller or hardware side effects."""
from . import config as cfg
import math
from .gait import calculate_trajectory, calc_dir
from .servos import correct_angle
from .kinematics import inverse_kinematics
from .body import pose_point

JOINT_NAMES = ("coxa", "femur", "tibia")


def calculate_frame(phase=0.0, ramp=0.0, direction=(0.0, 0.0), *,
                    step_length=None, step_height=None, z_height=None,
                    x_offset_front=None, y_offset_front=None,
                    x_offset_rear=None, y_offset_rear=None,
                    body_x=0.0, body_y=0.0, body_roll=0.0, body_pitch=0.0,
                    turn=0.0, turn_step_angle=None):
    """Body translation in mm: +body_x forward, +body_y right.

    Roll/pitch in degrees: positive lowers right/front. Invert the full body
    pose on foot targets to keep the world foot trajectory unchanged.
    """
    step_length = cfg.step_length if step_length is None else step_length
    step_height = cfg.step_height if step_height is None else step_height
    z_height = cfg.z_height if z_height is None else z_height
    turn_step_angle = cfg.turn_step_angle if turn_step_angle is None else turn_step_angle
    # Rotation axis through the center of the mounting positions.
    center_x = (min(p[0] for p in cfg.LEG_ORIGINS.values())
                + max(p[0] for p in cfg.LEG_ORIGINS.values())) / 2
    center_y = (min(p[1] for p in cfg.LEG_ORIGINS.values())
                + max(p[1] for p in cfg.LEG_ORIGINS.values())) / 2
    legs, servos = {}, {}
    for name, joints in cfg.LEGS.items():
        x_offset, y_offset = ((x_offset_front, y_offset_front) if name.endswith("f")
                              else (x_offset_rear, y_offset_rear))
        side = 1 if name.startswith("l") else -1
        start = (cfg.p_start[name][0] if x_offset is None else x_offset,
                 cfg.p_start[name][1] if y_offset is None else y_offset,
                 z_height)
        # Input direction: forward/right. Local +Y points outward on each leg.
        local_direction = (direction[0],
                           -direction[1] if name.startswith("l") else direction[1])
        leg_phase = (phase + cfg.LEG_PHASE_OFFSET[name]) % 1.0
        target = calculate_trajectory(
            leg_phase,
            step_length * ramp, step_height * ramp,
            start, *local_direction,
        )
        if turn:
            # Swing follows the turn; stance pushes in the opposite direction.
            progress = 2 * leg_phase - .5 if leg_phase < .5 else 1.5 - 2 * leg_phase
            angle = -math.radians(turn_step_angle) * turn * ramp * progress
            c, s = math.cos(angle), math.sin(angle)
            origin = cfg.LEG_ORIGINS[name]
            x, y = origin[0] + target[0] - center_x, origin[1] + side * target[1] - center_y
            target = (center_x + c * x - s * y - origin[0],
                      side * (center_y + s * x + c * y - origin[1]), target[2])
        if body_roll or body_pitch:
            origin = cfg.LEG_ORIGINS[name]
            world = (origin[0] + target[0], origin[1] + side * target[1],
                     origin[2] + target[2])
            local = pose_point(world, (body_x, -body_y, 0), body_roll, body_pitch,
                               inverse=True)
            target = (local[0] - origin[0], side * (local[1] - origin[1]),
                      local[2] - origin[2])
        else:
            target = (target[0] - body_x, target[1] + side * body_y, target[2])
        raw = inverse_kinematics(*target, cfg.l_coxa, cfg.l_femur, cfg.l_tibia)
        corrected = {}
        for joint, angle in zip(JOINT_NAMES, raw):
            sid = joints[joint][cfg.SERVO_ID]
            corrected[joint] = correct_angle(sid, angle)
            servos[sid] = corrected[joint]
        legs[name] = {"target": target, "raw_angles": raw, "servo_angles": corrected}
    return {"legs": legs, "servos": servos, "body_offset": (body_x, -body_y, 0.0),
            "body_rotation": (body_roll, body_pitch, 0.0)}


def advance_ramp(ramp, running, elapsed):
    ramp_step = elapsed / cfg.ramp_time
    ramp = min(1.0, ramp + ramp_step) if running else max(0.0, ramp - ramp_step)
    return ramp


def next_phase(phase, ramp, elapsed, *, gait_speed=None):
    gait_speed = cfg.gait_speed if gait_speed is None else gait_speed
    return (phase + gait_speed * elapsed) % 1.0 if ramp > 0.0 else 0.0


def initial_state():
    """Nowy, niezależny stan ruchu."""
    return {"phase": 0.0, "ramp": 0.0, "direction": (0.0, 0.0), "turn": 0.0}


def step_motion(state, x, y, elapsed, *, gait_speed=None, step_length=None,
                step_height=None, z_height=None,
                x_offset_front=None, y_offset_front=None,
                x_offset_rear=None, y_offset_rear=None,
                body_x=0.0, body_y=0.0, body_roll=0.0, body_pitch=0.0,
                turn=0.0, turn_step_angle=None):
    """Zwraca (nowy stan, klatka), nie modyfikując przekazanego stanu.

    Klatka używa bieżącej fazy i nowej rampy; zwrócony stan zawiera
    fazę następnego kroku. Przy hamowaniu zachowujemy ostatni kierunek.
    """
    translating = math.hypot(x, y) > cfg.stick_deadzone
    turning = abs(turn) > cfg.stick_deadzone
    running = translating or turning
    direction = calc_dir(x, y) if translating else ((0.0, 0.0) if running else tuple(state["direction"]))
    active_turn = turn if turning else (0.0 if running else state.get("turn", 0.0))
    ramp = advance_ramp(state["ramp"], running, elapsed)
    frame = calculate_frame(state["phase"], ramp, direction,
                            step_length=step_length, step_height=step_height,
                            z_height=z_height,
                            x_offset_front=x_offset_front, y_offset_front=y_offset_front,
                            x_offset_rear=x_offset_rear, y_offset_rear=y_offset_rear,
                            body_x=body_x, body_y=body_y,
                            body_roll=body_roll, body_pitch=body_pitch,
                            turn=active_turn, turn_step_angle=turn_step_angle)
    updated = {"phase": next_phase(state["phase"], ramp, elapsed, gait_speed=gait_speed),
               "ramp": ramp, "direction": direction, "turn": active_turn}
    return updated, frame
