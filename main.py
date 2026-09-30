import socket, json, math, time
from config import (LEGS, SERVO_ID, LEG_PHASE_OFFSET, l_coxa, l_femur, l_tibia, gait_speed, step_length, step_height,
                     p_start, ramp_time, stick_deadzone)
from ik import inverse_kinematics
from gait import calculate_trajectory, calc_dir, correct_angle
from joystick import PS4Controller

ESP = ("192.168.0.115", 8888)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ACTIVE_SERVO_IDS = {1, 2, 3, 4, 5, 6}

controller = PS4Controller()

# --- Stan globalny modułu (jak w wersji na ESP32: running, gait_phase, itd.) ---
running = False
gait_phase = 0.0
ramp = 0.0
last_dir = (0.0, 0.0)

last_up = last_down = last_left = last_right = False

last_send_time = time.time()
last_phase_time = time.time()


def send_servos(angles: dict):
    if not angles:
        return
    payload = {"set_servo": {str(k): round(v, 1) for k, v in angles.items()}}
    sock.sendto(json.dumps(payload).encode(), ESP)


def return_to_neutral():
    """Ustawia wszystkie nogi w pozycji spoczynkowej P_START, jednym pakietem."""
    global running, gait_phase, ramp
    angles_to_send = {}
    for leg in LEGS:
        p = p_start[leg]
        coxa_angle, femur_angle, tibia_angle = inverse_kinematics(p[0], p[1], p[2], l_coxa, l_femur, l_tibia)
        for joint, angle in (("coxa", coxa_angle), ("femur", femur_angle), ("tibia", tibia_angle)):
            sid = LEGS[leg][joint][SERVO_ID]
            if sid in ACTIVE_SERVO_IDS:
                angles_to_send[sid] = correct_angle(sid, angle)
    send_servos(angles_to_send)
    running = False
    gait_phase = 0.0
    ramp = 0.0


def execute_gait(elapsed):
    """Liczy jedną klatkę chodu (rampa + faza + IK) i wysyła do ESP."""
    global gait_phase, ramp

    ramp_step = elapsed / ramp_time
    ramp = min(1.0, ramp + ramp_step) if running else max(0.0, ramp - ramp_step)

    cur_step_length = step_length * ramp
    cur_step_height = step_height * ramp

    angles_to_send = {}
    for leg in LEGS:
        leg_start = p_start[leg]
        leg_phase = (gait_phase + LEG_PHASE_OFFSET[leg]) % 1.0
        foot_pos = calculate_trajectory(leg_phase, cur_step_length, cur_step_height,
                                         leg_start, last_dir[0], last_dir[1])
        coxa_angle, femur_angle, tibia_angle = inverse_kinematics(
            foot_pos[0], foot_pos[1], foot_pos[2], l_coxa, l_femur, l_tibia
        )
        for joint, angle in (("coxa", coxa_angle), ("femur", femur_angle), ("tibia", tibia_angle)):
            sid = LEGS[leg][joint][SERVO_ID]
            if sid in ACTIVE_SERVO_IDS:
                angles_to_send[sid] = correct_angle(sid, angle)

    send_servos(angles_to_send)
    print(f"gait_phase: {gait_phase:.3f}, ramp: {ramp:.3f}, angles: {angles_to_send}")
    gait_phase = (gait_phase + gait_speed * elapsed) % 1.0 if ramp > 0.0 else 0.0


def process_ps4_input():
    """Czyta lewy joystick, ustawia running/last_dir. Miejsce na prawy stick w przyszłości."""
    global running, last_dir

    stick_x, stick_y = controller.get_left_stick()
    jx, jy = stick_y, stick_x
    magnitude = math.sqrt(jx**2 + jy**2)

    if magnitude > stick_deadzone:
        running = True
        last_dir = calc_dir(jx, jy)
        return

    # rx, ry = controller.get_right_stick()
    # if math.sqrt(rx**2 + ry**2) > stick_deadzone:
    #     running = False  # np. tryb tilt/rotate zamiast chodu
    #     ... obsługa prawego sticka ...
    #     return

    running = False


# def process_ps4_buttons():
#     """Odczytuje przyciski, na zboczu narastającym wykonuje akcje."""
#     global last_up, last_down, last_left, last_right

#     up = controller.get_button("up")
#     down = controller.get_button("down")
#     left = controller.get_button("left")
#     right = controller.get_button("right")

#     if up and not last_up:
#         pass  # np. zmiana wysokości korpusu

#     if down and not last_down:
#         pass

#     if left and not last_left:
#         pass  # np. zmiana t_cycle / prędkości chodu

#     if right and not last_right:
#         pass

#     last_up, last_down, last_left, last_right = up, down, left, right


def main_loop():
    global last_phase_time

    return_to_neutral()
    last_phase_time = time.time()

    while True:
        current_time = time.time()
        elapsed = current_time - last_phase_time
        last_phase_time = current_time

        process_ps4_input()
        # process_ps4_buttons()

        if running or ramp > 0.0:
            execute_gait(elapsed)
        else:
            return_to_neutral()

        time.sleep(0.02)

if __name__ == "__main__":
    return_to_neutral()
    main_loop()