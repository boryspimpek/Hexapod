"""Uruchomienie sterowania robotem: python main.py."""
import time
from robot import config as cfg
from robot.joystick import (open_controller, get_left_stick, get_right_stick,
                            get_dpad_vertical, close_controller)
from robot.motion import calculate_frame, initial_state, step_motion
from robot.transport import open_socket, send_servos


def send_frame(sock, frame):
    angles = {sid: angle for sid, angle in frame["servos"].items()
              if sid in cfg.ACTIVE_SERVO_IDS}
    send_servos(sock, angles, precision=1)
    return angles


def main_loop(controller, sock):
    state = initial_state()
    z_height = cfg.z_height
    body_shift = (0.0, 0.0)
    previous_dpad = 0
    send_frame(sock, calculate_frame())
    last_time = time.monotonic()
    while True:
        current_time = time.monotonic()
        elapsed = current_time - last_time
        last_time = current_time
        stick_x, stick_y = get_left_stick(controller)
        right_x, right_y = get_right_stick(controller)
        requested_shift = (right_y * cfg.BODY_SHIFT_MAX, right_x * cfg.BODY_SHIFT_MAX)
        dpad = get_dpad_vertical(controller)
        requested_height = z_height
        if dpad and dpad != previous_dpad:
            lo, hi = cfg.HEIGHT_LIMITS
            requested_height = max(lo, min(hi, z_height - dpad * cfg.HEIGHT_STEP))
        previous_dpad = dpad
        phase = state["phase"]
        was_moving = state["ramp"] > 0
        try:
            next_state, frame = step_motion(state, stick_y, stick_x, elapsed,
                                            z_height=requested_height,
                                            body_x=requested_shift[0], body_y=requested_shift[1])
        except ValueError:
            if requested_height == z_height and requested_shift == body_shift:
                raise
            print("Zmiana pozycji korpusu odrzucona: cel poza zasiegiem nogi.")
            next_state, frame = step_motion(state, stick_y, stick_x, elapsed,
                                            z_height=z_height,
                                            body_x=body_shift[0], body_y=body_shift[1])
            requested_height = z_height
            requested_shift = body_shift
        body_shift = requested_shift
        if requested_height != z_height:
            z_height = requested_height
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
