# Configuration file for hexapod robot parameters

# Lengths of the leg segments in millimeters
l_coxa, l_femur, l_tibia = 40.0, 80.0, 120.0

# (id, leg, joint, invert_multiplier)
SERVO_MAPPING = [
    (1, 'lf', 'coxa', -1), (2, 'lf', 'femur', -1), (3, 'lf', 'tibia', -1),  # Serwo 1: LF Coxa, Serwo 2: LF Femur, Serwo 3: LF Tibia
    (4, 'rf', 'coxa', 1), (5, 'rf', 'femur', 1), (6, 'rf', 'tibia', 1),     # Serwo 4: RF Coxa, Serwo 5: RF Femur, Serwo 6: RF Tibia
    (7, 'lr', 'coxa', -1), (8, 'lr', 'femur', -1), (9, 'lr', 'tibia', -1),  # Serwo 7: LR Coxa, Serwo 8: LR Femur, Serwo 9: LR Tibia
    (10, 'rr', 'coxa', 1), (11, 'rr', 'femur', 1), (12, 'rr', 'tibia', 1)   # Serwo 10: RR Coxa, Serwo 11: RR Femur, Serwo 12: RR Tibia
]

SERVO_LIMITS = {
    1: (0, 180), 2: (0, 180), 3: (0, 180), 4: (0, 180), 5: (0, 180), 6: (0, 180),
    7: (0, 180), 8: (0, 180), 9: (0, 180), 10: (0, 180), 11: (0, 180), 12: (0, 180)
}

SERVO_TRIMS = {
    1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0,
    7: 0, 8: 0, 9: 0, 10: 0, 11: 0, 12: 0
}