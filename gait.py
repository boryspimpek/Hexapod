import socket, json
import math
import time
from config import LEGS, SERVO_ID, LEG_PHASE_OFFSET, INVERTED, LIMITS, TRIM, l_coxa, l_femur, l_tibia, gait_speed, step_length, step_height, p_start
from ik import inverse_kinematics
from joystick import PS4Controller

ESP = ("192.168.0.115", 8888)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Serwa, które faktycznie są podpięte do ESP32 (dopisuj kolejne w miarę rozbudowy)
ACTIVE_SERVO_IDS = {10, 11, 12}

controller = PS4Controller()

# Słownik: id serwa -> dane stawu (budowany raz)
JOINTS = {}
for _leg, _joints in LEGS.items():
    for _name, _data in _joints.items():
        JOINTS[_data[SERVO_ID]] = _data


def calc_dir(jx, jy):
    magnitude = math.sqrt(jx**2 + jy**2)
    if magnitude < 1e-6:
        return (0.0, 0.0)
    return (jx / magnitude, jy / magnitude)


def calculate_trajectory(global_phase, step_length, step_height, p_start, jx, jy):
    swing_ratio = 0.5
    step_dir = calc_dir(jx, jy)

    p_end = (
        p_start[0] + step_dir[0] * step_length,
        p_start[1] + step_dir[1] * step_length,
        p_start[2]
    )

    if global_phase < swing_ratio:
        t = global_phase / swing_ratio
        pos_x = p_start[0] + (p_end[0] - p_start[0]) * t
        pos_y = p_start[1] + (p_end[1] - p_start[1]) * t
        pos_z = p_start[2] + step_height * math.sin(t * math.pi)
    else:
        t = (global_phase - swing_ratio) / (1.0 - swing_ratio)
        pos_x = p_end[0] + (p_start[0] - p_end[0]) * t
        pos_y = p_end[1] + (p_start[1] - p_end[1]) * t
        pos_z = p_start[2]

    return (pos_x, pos_y, pos_z), (p_start, p_end)


def correct_angle(servo_id: int, angle_deg: float) -> float:
    """Odwrócenie, trim i limity. Zwraca kąt gotowy do wysłania."""
    data = JOINTS[servo_id]
    limits = data[LIMITS]

    angle = (180 - angle_deg) if data[INVERTED] else angle_deg
    angle += data[TRIM]
    return max(limits[0], min(limits[1], angle))


def send_servos(angles: dict):
    """Wysyła wszystkie kąty jednym pakietem UDP, bez czekania na odpowiedź."""
    if not angles:
        return
    payload = {"set_servo": {str(k): round(v, 1) for k, v in angles.items()}}
    sock.sendto(json.dumps(payload).encode(), ESP)


def main_loop():
    global_time_phase = 0.0
    dt = 0.02  # 50 Hz

    while True:
        stick_x, stick_y = controller.get_left_stick()
        jx = stick_y
        jy = stick_x

        angles_to_send = {}

        for leg in LEGS:
            leg_phase = (global_time_phase + LEG_PHASE_OFFSET[leg]) % 1.0
            foot_pos, _ = calculate_trajectory(
                leg_phase, step_length, step_height, p_start, jx, jy
            )
            coxa_angle, femur_angle, tibia_angle = inverse_kinematics(
                foot_pos[0], foot_pos[1], foot_pos[2], l_coxa, l_femur, l_tibia
            )

            for joint, angle in (("coxa", coxa_angle), ("femur", femur_angle), ("tibia", tibia_angle)):
                sid = LEGS[leg][joint][SERVO_ID]
                if sid in ACTIVE_SERVO_IDS:
                    angles_to_send[sid] = correct_angle(sid, angle)

        send_servos(angles_to_send)
        print(angles_to_send)

        global_time_phase = (global_time_phase + gait_speed * dt) % 1.0
        time.sleep(dt)


if __name__ == "__main__":
    main_loop()