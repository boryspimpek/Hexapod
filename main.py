"""Uruchomienie sterowania robotem: python main.py."""
from dataclasses import dataclass
import math
import time
from robot import config as cfg
from robot.joystick import (open_controller, get_left_stick, get_right_stick,
                            get_dpad_vertical, get_r1, get_gait_button,
                            get_rest_button, get_walk_button, close_controller)
from robot.gait import next_gait
from robot.motion import JOINT_NAMES, calculate_frame, initial_state, step_motion
from robot.servos import apply_offsets
from robot.transport import open_socket, send_servos

SERVO_LIMIT_REPORT_INTERVAL = 1.0


@dataclass(frozen=True)
class MotionCommand:
    """Wartości sterujące gotowe do przekazania do modelu ruchu."""
    x: float
    y: float
    turn: float
    z_height: float
    body_x: float
    body_y: float
    body_roll: float
    body_pitch: float
    slow_pose_transition: bool = False


@dataclass
class ControlState:
    """Ostatnie zaakceptowane ustawienia i krawędzie sterowania."""
    z_height: float = cfg.z_height
    body_shift: tuple = (0.0, 0.0)
    body_tilt: tuple = (0.0, 0.0)
    previous_dpad: int = 0
    previous_rest_button: bool = False


def smooth_toward(current, target, elapsed, time_constant):
    """Wygładź zmianę wartości z filtrem wykładniczym."""
    if elapsed <= 0.0 or time_constant <= 0.0:
        return target
    blend = 1.0 - math.exp(-elapsed / time_constant)
    return current + (target - current) * blend


def read_motion_command(controller, state, controls, elapsed=cfg.LOOP_INTERVAL):
    """Odczytaj sterowanie i zmapuj je na komendę ruchu."""
    stick_x, stick_y = get_left_stick(controller)
    right_x, right_y = get_right_stick(controller)
    shifting = get_r1(controller)

    target_body_x, target_body_y = (
        (right_y * cfg.BODY_SHIFT_MAX, right_x * cfg.BODY_SHIFT_MAX)
        if shifting else (0.0, 0.0))
    turn = 0.0 if shifting else right_x
    target_body_roll, target_body_pitch = (
        (stick_x * cfg.BODY_TILT_MAX, stick_y * cfg.BODY_TILT_MAX)
        if shifting else (0.0, 0.0))
    smoothing = cfg.BODY_POSE_SMOOTHING_SECONDS
    body_x = smooth_toward(controls.body_shift[0], target_body_x, elapsed, smoothing)
    body_y = smooth_toward(controls.body_shift[1], target_body_y, elapsed, smoothing)
    body_roll = smooth_toward(controls.body_tilt[0], target_body_roll, elapsed, smoothing)
    body_pitch = smooth_toward(controls.body_tilt[1], target_body_pitch, elapsed, smoothing)
    if shifting:
        state = {**state, "turn": 0.0}
        stick_x = stick_y = 0.0

    dpad = get_dpad_vertical(controller)
    requested_height = controls.z_height
    if dpad and dpad != controls.previous_dpad:
        lo, hi = cfg.HEIGHT_LIMITS
        requested_height = max(lo, min(hi, controls.z_height - dpad * cfg.HEIGHT_STEP))
    controls.previous_dpad = dpad

    rest_button = get_rest_button(controller)
    slow_pose_transition = False
    if rest_button and not controls.previous_rest_button:
        requested_height = cfg.REST_HEIGHT
        slow_pose_transition = True
    controls.previous_rest_button = rest_button

    command = MotionCommand(
        x=stick_x, y=stick_y, turn=turn, z_height=requested_height,
        body_x=body_x, body_y=body_y,
        body_roll=body_roll, body_pitch=body_pitch,
        slow_pose_transition=slow_pose_transition,
    )
    return command, state


def send_servo_angles(sock, angles):
    active_angles = {sid: angle for sid, angle in angles.items()
                     if sid in cfg.ACTIVE_SERVO_IDS}
    send_servos(sock, active_angles, precision=1)
    return active_angles


def send_frame(sock, frame):
    return send_servo_angles(sock, frame["servos"])


def interpolate_servo_angles(start_angles, target_angles, progress):
    """Interpoluj kąty z łagodnym startem i zatrzymaniem."""
    progress = max(0.0, min(1.0, progress))
    eased = progress * progress * (3.0 - 2.0 * progress)
    return {
        sid: start_angles.get(sid, target) + (target - start_angles.get(sid, target)) * eased
        for sid, target in target_angles.items()
    }


def find_limited_servo_angles(frame):
    """Zwróć kąty, które zostały obcięte do limitu serwa."""
    limited = []
    for leg_name, leg in frame["legs"].items():
        for joint, raw_angle in zip(JOINT_NAMES, leg["raw_angles"]):
            servo_id = cfg.LEGS[leg_name][joint][cfg.SERVO_ID]
            if servo_id not in cfg.ACTIVE_SERVO_IDS:
                continue
            requested = apply_offsets(servo_id, raw_angle)
            limited_angle = leg["servo_angles"][joint]
            if abs(requested - limited_angle) > 1e-8:
                lo, hi = cfg.LEGS[leg_name][joint][cfg.LIMITS]
                limited.append((servo_id, leg_name, joint, requested, limited_angle, lo, hi))
    return limited


def calculate_motion_step(state, command, elapsed, controls):
    """Oblicz klatkę; przy niedostępnej pozycji korpusu zachowaj poprzednią."""
    requested_shift = (command.body_x, command.body_y)
    requested_tilt = (command.body_roll, command.body_pitch)
    try:
        next_state, frame = step_motion(
            state, command.y, command.x, elapsed,
            z_height=command.z_height,
            body_x=command.body_x, body_y=command.body_y,
            body_roll=command.body_roll, body_pitch=command.body_pitch,
            turn=command.turn)
        return next_state, frame, command.z_height, requested_shift, requested_tilt
    except ValueError:
        if (command.z_height == controls.z_height
                and requested_shift == controls.body_shift
                and requested_tilt == controls.body_tilt):
            raise

        print("Zmiana pozycji korpusu odrzucona: cel poza zasiegiem nogi.")
        next_state, frame = step_motion(
            state, command.y, command.x, elapsed,
            z_height=controls.z_height,
            body_x=controls.body_shift[0], body_y=controls.body_shift[1],
            body_roll=controls.body_tilt[0], body_pitch=controls.body_tilt[1],
            turn=command.turn)
        return next_state, frame, controls.z_height, controls.body_shift, controls.body_tilt


def main_loop(controller, sock):
    state = initial_state()
    controls = ControlState()
    previous_gait_button = False
    previous_walk_button = False
    last_angles = send_frame(sock, calculate_frame())
    transition_start_angles = None
    transition_start_time = None
    last_time = time.monotonic()

    # Zbieranie zdarzeń limitów w całym interwale raportowania.
    last_limit_report = last_time
    limit_events = {}   # servo_id -> statystyki z bieżącego interwału
    limit_frames = 0    # liczba klatek w bieżącym interwale

    while True:
        current_time = time.monotonic()
        elapsed = current_time - last_time
        last_time = current_time
        command, state = read_motion_command(controller, state, controls, elapsed)
        gait_button = get_gait_button(controller)
        if gait_button and not previous_gait_button:
            state = {**state, "requested_gait": next_gait(state["requested_gait"])}
            print(f"Wybrano gait: {state['requested_gait']}")
        previous_gait_button = gait_button
        walk_button = get_walk_button(controller)
        reset_to_initial = walk_button and not previous_walk_button
        previous_walk_button = walk_button
        if reset_to_initial:
            state = initial_state()
            command = MotionCommand(
                x=0.0, y=0.0, turn=0.0, z_height=cfg.z_height,
                body_x=0.0, body_y=0.0, body_roll=0.0, body_pitch=0.0,
                slow_pose_transition=True,
            )
        phase = state["phase"]
        was_moving = state["ramp"] > 0
        next_state, frame, accepted_height, accepted_shift, accepted_tilt = (
            calculate_motion_step(state, command, elapsed, controls))

        # Zbieramy limity z każdej klatki, a raportujemy raz na interwał.
        limit_frames += 1
        for servo_id, leg_name, joint, requested, limited, lo, hi in find_limited_servo_angles(frame):
            overshoot = abs(requested - limited)
            ev = limit_events.get(servo_id)
            if ev is None:
                limit_events[servo_id] = {
                    "leg": leg_name, "joint": joint, "count": 1,
                    "max": overshoot, "requested": requested,
                    "limited": limited, "lo": lo, "hi": hi,
                }
            else:
                ev["count"] += 1
                if overshoot > ev["max"]:
                    ev.update(max=overshoot, requested=requested, limited=limited)

        if current_time - last_limit_report >= SERVO_LIMIT_REPORT_INTERVAL:
            if limit_events:
                details = "; ".join(
                    f"ID {sid} ({ev['leg']}/{ev['joint']}): "
                    f"max {ev['requested']:.1f}° -> {ev['limited']:.1f}° "
                    f"(limit {ev['lo']}–{ev['hi']}°), {ev['count']}/{limit_frames} klatek"
                    for sid, ev in sorted(limit_events.items()))
                print(f"Serwa ograniczone do limitów: {details}")
            limit_events.clear()
            limit_frames = 0
            last_limit_report = current_time

        controls.body_shift = accepted_shift
        controls.body_tilt = accepted_tilt
        if accepted_height != controls.z_height:
            controls.z_height = accepted_height
            print(f"z_height: {controls.z_height:.1f} mm")
        if reset_to_initial:
            print("Przywrócono pozycję początkową.")
        state = next_state
        if command.slow_pose_transition or reset_to_initial:
            transition_start_angles = last_angles.copy()
            transition_start_time = current_time
        if transition_start_angles is None:
            angles = send_frame(sock, frame)
        else:
            duration = max(0.0, cfg.POSE_TRANSITION_SECONDS)
            progress = (1.0 if duration == 0.0 else
                        (current_time - transition_start_time) / duration)
            angles = send_servo_angles(
                sock, interpolate_servo_angles(transition_start_angles, frame["servos"], progress))
            if progress >= 1.0:
                transition_start_angles = None
                transition_start_time = None
        last_angles = angles
        # if was_moving or state["ramp"] > 0:
        #     print(f"gait_phase: {phase:.3f}, ramp: {state['ramp']:.3f}, angles: {angles}")
        time.sleep(cfg.LOOP_INTERVAL)

def main():
    controller = open_controller()
    try:
        with open_socket() as sock:
            main_loop(controller, sock)
    except KeyboardInterrupt:
        pass
    finally:
        close_controller(controller)


if __name__ == "__main__":
    main()
