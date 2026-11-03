"""Uruchomienie sterowania robotem: python main.py."""
from dataclasses import dataclass
import time
from robot import config as cfg
from robot.joystick import (open_controller, get_left_stick, get_right_stick,
                            get_dpad_vertical, get_r1, get_gait_button, close_controller)
from robot.gait import next_gait
from robot.motion import calculate_frame, initial_state, step_motion
from robot.transport import open_socket, send_servos


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


def read_motion_command(controller, state, z_height, previous_dpad):
    """Odczytaj drążki i D-pad, a następnie zmapuj je na komendę ruchu."""
    stick_x, stick_y = get_left_stick(controller)
    right_x, right_y = get_right_stick(controller)
    shifting = get_r1(controller)

    body_x, body_y = ((right_y * cfg.BODY_SHIFT_MAX, right_x * cfg.BODY_SHIFT_MAX)
                      if shifting else (0.0, 0.0))
    turn = 0.0 if shifting else right_x
    body_roll, body_pitch = ((stick_x * cfg.BODY_TILT_MAX, stick_y * cfg.BODY_TILT_MAX)
                             if shifting else (0.0, 0.0))
    if shifting:
        state = {**state, "turn": 0.0}
        stick_x = stick_y = 0.0

    dpad = get_dpad_vertical(controller)
    requested_height = z_height
    if dpad and dpad != previous_dpad:
        lo, hi = cfg.HEIGHT_LIMITS
        requested_height = max(lo, min(hi, z_height - dpad * cfg.HEIGHT_STEP))

    command = MotionCommand(
        x=stick_x, y=stick_y, turn=turn, z_height=requested_height,
        body_x=body_x, body_y=body_y,
        body_roll=body_roll, body_pitch=body_pitch,
    )
    return command, state, dpad


def send_frame(sock, frame):
    angles = {sid: angle for sid, angle in frame["servos"].items()
              if sid in cfg.ACTIVE_SERVO_IDS}
    send_servos(sock, angles, precision=1)
    return angles


def main_loop(controller, sock):
    state = initial_state()
    z_height = cfg.z_height
    body_shift = (0.0, 0.0)
    body_tilt = (0.0, 0.0)
    previous_dpad = 0
    previous_gait_button = False
    send_frame(sock, calculate_frame())
    last_time = time.monotonic()
    while True:
        current_time = time.monotonic()
        elapsed = current_time - last_time
        last_time = current_time
        command, state, previous_dpad = read_motion_command(
            controller, state, z_height, previous_dpad)
        gait_button = get_gait_button(controller)
        if gait_button and not previous_gait_button:
            state = {**state, "requested_gait": next_gait(state["requested_gait"])}
            print(f"Wybrano gait: {state['requested_gait']}")
        previous_gait_button = gait_button
        requested_shift = (command.body_x, command.body_y)
        requested_tilt = (command.body_roll, command.body_pitch)
        phase = state["phase"]
        was_moving = state["ramp"] > 0
        accepted_height = command.z_height
        try:
            next_state, frame = step_motion(
                state, command.y, command.x, elapsed,
                z_height=command.z_height,
                body_x=command.body_x, body_y=command.body_y,
                body_roll=command.body_roll, body_pitch=command.body_pitch,
                turn=command.turn)
        except ValueError:
            if (command.z_height == z_height and requested_shift == body_shift
                    and requested_tilt == body_tilt):
                raise
            print("Zmiana pozycji korpusu odrzucona: cel poza zasiegiem nogi.")
            next_state, frame = step_motion(
                state, command.y, command.x, elapsed,
                z_height=z_height,
                body_x=body_shift[0], body_y=body_shift[1],
                body_roll=body_tilt[0], body_pitch=body_tilt[1], turn=command.turn)
            accepted_height = z_height
            requested_shift = body_shift
            requested_tilt = body_tilt
        body_shift = requested_shift
        body_tilt = requested_tilt
        if accepted_height != z_height:
            z_height = accepted_height
            print(f"z_height: {z_height:.1f} mm")
        state = next_state
        angles = send_frame(sock, frame)
        if was_moving or state["ramp"] > 0:
            print(f"gait_phase: {phase:.3f}, ramp: {state['ramp']:.3f}, angles: {angles}")
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
