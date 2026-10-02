import time
from config import LEGS, SERVO_ID, INVERTED, LIMITS, TRIM

def find_joint_by_servo_id(servo_id: int):
    for leg, joints in LEGS.items():
        for joint_name, data in joints.items():
            if data[SERVO_ID] == servo_id:
                return leg, joint_name, data
    raise ValueError(f"Nie znaleziono serwa o ID {servo_id}")


def set_servo_angle(servo_id: int, angle_deg: float):
    leg, joint_name, data = find_joint_by_servo_id(servo_id)
    inverted = data[INVERTED]
    limits = data[LIMITS]
    trim = data[TRIM]

    corrected_angle = (180 - angle_deg) if inverted else angle_deg
    corrected_angle += trim
    corrected_angle = max(limits[0], min(limits[1], corrected_angle))
