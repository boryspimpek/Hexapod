from time import time
from config import LEGS, SERVO_ID, LEG_PHASE_OFFSET, l_coxa, l_femur, l_tibia
from gait import calculate_trajectory
from ik import inverse_kinematics
from joystick import PS4Controller
from move_servo import set_servo_angle

controller = PS4Controller()

gait_speed = 0.5  # ile "cykli chodu" na sekundę — to jest Twoja "prędkość"
step_length = 50.0  # długość kroku w mm
step_height = 20.0  # wysokość unoszenia stopy w mm
p_start = (0, 160, -50)

def main_loop():
    global_time_phase = 0.0
    dt = 0.02  # 20 ms = 50 Hz, częstość odświeżania serwomechanizmów i odczytu joysticka


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
            
            set_servo_angle(LEGS[leg]['coxa'][SERVO_ID], coxa_angle)
            set_servo_angle(LEGS[leg]['femur'][SERVO_ID], femur_angle)
            set_servo_angle(LEGS[leg]['tibia'][SERVO_ID], tibia_angle)
        
        global_time_phase = (global_time_phase + gait_speed * dt) % 1.0
        time.sleep(dt)