import time
import board
import busio
from adafruit_pca9685 import PCA9685
from config import LEGS, SERVO_ID, INVERTED, LIMITS, TRIM

# Inicjalizacja I2C i sterownika PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # standardowa częstotliwość dla serw hobbystycznych

# Typowy zakres impulsu serwa w mikrosekundach (dopasuj do swoich serw!)
SERVO_MIN_US = 500
SERVO_MAX_US = 2500


def angle_to_duty_cycle(angle_deg: float) -> int:
    pulse_us = SERVO_MIN_US + (angle_deg / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
    period_us = 1_000_000 / pca.frequency  # np. 20000 us przy 50 Hz
    duty_cycle = int((pulse_us / period_us) * 0xFFFF)
    return duty_cycle


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

    duty_cycle = angle_to_duty_cycle(corrected_angle)
    pca.channels[servo_id - 1].duty_cycle = duty_cycle

    print(f"{leg}.{joint_name} (servo {servo_id}) -> {corrected_angle:.1f}° (duty={duty_cycle})")