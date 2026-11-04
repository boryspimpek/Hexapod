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


@dataclass
class ControlState:
    """Ostatnie zaakceptowane ustawienia i krawędzie sterowania."""
    z_height: float = cfg.z_height
    body_shift: tuple = (0.0, 0.0)
    body_tilt: tuple = (0.0, 0.0)
    previous_dpad: int = 0


def read_motion_command(controller, state, controls):
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
    requested_height = controls.z_height
    if dpad and dpad != controls.previous_dpad:
        lo, hi = cfg.HEIGHT_LIMITS
        requested_height = max(lo, min(hi, controls.z_height - dpad * cfg.HEIGHT_STEP))
    controls.previous_dpad = dpad

    command = MotionCommand(
        x=stick_x, y=stick_y, turn=turn, z_height=requested_height,
        body_x=body_x, body_y=body_y,
        body_roll=body_roll, body_pitch=body_pitch,
    )
    return command, state


def send_frame(sock, frame):
    angles = {sid: angle for sid, angle in frame["servos"].items()
              if sid in cfg.ACTIVE_SERVO_IDS}
    send_servos(sock, angles, precision=1)
    return angles


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
    send_frame(sock, calculate_frame())
    last_time = time.monotonic()
    while True:
        current_time = time.monotonic()
        elapsed = current_time - last_time
        last_time = current_time
        command, state = read_motion_command(controller, state, controls)
        gait_button = get_gait_button(controller)
        if gait_button and not previous_gait_button:
            state = {**state, "requested_gait": next_gait(state["requested_gait"])}
            print(f"Wybrano gait: {state['requested_gait']}")
        previous_gait_button = gait_button
        phase = state["phase"]
        was_moving = state["ramp"] > 0
        next_state, frame, accepted_height, accepted_shift, accepted_tilt = (
            calculate_motion_step(state, command, elapsed, controls))
        controls.body_shift = accepted_shift
        controls.body_tilt = accepted_tilt
        if accepted_height != controls.z_height:
            controls.z_height = accepted_height
            print(f"z_height: {controls.z_height:.1f} mm")
        state = next_state
        angles = send_frame(sock, frame)
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
