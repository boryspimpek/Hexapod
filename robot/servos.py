"""Kalibracja i ograniczenia kątów serw."""
from .config import LEGS, SERVO_ID, INVERTED, LIMITS, TRIM

JOINTS = {d[SERVO_ID]: d for joints in LEGS.values() for d in joints.values()}

def apply_offsets(servo_id, angle_deg):
    """Inwersja + trim, bez limitów."""
    data = JOINTS[servo_id]
    a = (180 - angle_deg) if data[INVERTED] else angle_deg
    return a + data[TRIM]


def correct_angle(servo_id, angle_deg):
    """Kąt gotowy do wysyłki: inwersja, trim, limity."""
    return limit_angle(servo_id, apply_offsets(servo_id, angle_deg))

def limit_angle(servo_id, angle_deg):
    """Ogranicza już skalibrowany kąt komendy do zakresu serwa."""
    lo, hi = JOINTS[servo_id][LIMITS]
    return max(lo, min(hi, angle_deg))


def remove_offsets(servo_id, command):
    """Odtwarza kąt w konwencji IK/FK z komendy serwa."""
    data = JOINTS[servo_id]
    angle = command - data[TRIM]
    return 180 - angle if data[INVERTED] else angle
