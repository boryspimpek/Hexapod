import math
import numpy as np
import time
from config import LEGS, SERVO_ID, LEG_PHASE_OFFSET, l_coxa, l_femur, l_tibia, gait_speed, step_length, step_height, p_start
from ik import inverse_kinematics
from move_servo import set_servo_angle
from joystick import PS4Controller

controller = PS4Controller()

def calc_dir(jx, jy):
    magnitude = math.sqrt(jx**2 + jy**2)
    if magnitude < 1e-6:
        # drążek w pozycji neutralnej -> brak kierunku ruchu
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


def main_loop():
    global_time_phase = 0.0
    dt = 1  # 20 ms = 50 Hz, częstość odświeżania serwomechanizmów i odczytu joysticka

    while True:
        stick_x, stick_y = controller.get_left_stick()
        jx = stick_y
        jy = stick_x

        for leg in LEGS:
            leg_phase = (global_time_phase + LEG_PHASE_OFFSET[leg]) % 1.0
            foot_pos, _ = calculate_trajectory(
                leg_phase, step_length, step_height, p_start, jx, jy
            )
            coxa_angle, femur_angle, tibia_angle = inverse_kinematics(foot_pos[0], foot_pos[1], foot_pos[2], l_coxa, l_femur, l_tibia)

            # print(
            #     f"Leg {leg}: Foot pos ({foot_pos[0]:.2f}, {foot_pos[1]:.2f}, {foot_pos[2]:.2f}), "
            #     f"Angles: Coxa {coxa_angle:.2f}, Femur {femur_angle:.2f}, Tibia {tibia_angle:.2f}"
            # )

            set_servo_angle(LEGS[leg]['coxa'][SERVO_ID], coxa_angle)
            set_servo_angle(LEGS[leg]['femur'][SERVO_ID], femur_angle)
            set_servo_angle(LEGS[leg]['tibia'][SERVO_ID], tibia_angle)

        print("================================================================================================")
        
        global_time_phase = (global_time_phase + gait_speed * dt) % 1.0
        time.sleep(dt)
        
if __name__ == "__main__":
    main_loop()