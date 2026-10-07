"""Uruchomienie sterowania robotem: python main.py."""
import time
from robot import config as cfg
from robot.joystick import open_controller, get_left_stick, get_dpad_vertical, close_controller
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
    previous_dpad = 0
    send_frame(sock, calculate_frame())
    last_time = time.monotonic()
    while True:
        current_time = time.monotonic()
        elapsed = current_time - last_time
        last_time = current_time
        stick_x, stick_y = get_left_stick(controller)
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
                                            z_height=requested_height)
        except ValueError:
            if requested_height == z_height:
                raise
            print("Zmiana wysokosci odrzucona: cel poza zasiegiem nogi.")
            next_state, frame = step_motion(state, stick_y, stick_x, elapsed,
                                            z_height=z_height)
            requested_height = z_height
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
