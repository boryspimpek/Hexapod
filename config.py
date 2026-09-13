# Configuration file for quadruped robot parameters

l_coxa, l_femur, l_tibia = 40.0, 80.0, 120.0

# Indeksy pól w krotce (servo_id, inverted, limits, trim)
SERVO_ID, INVERTED, LIMITS, TRIM = 0, 1, 2, 3

LEGS = {
    'lf': {
        'coxa':  (1,  True,  (0, 180), 0.0),
        'femur': (2,  True,  (0, 180), 0.0),
        'tibia': (3,  True,  (0, 180), 0.0),
    },
    'rf': {
        'coxa':  (4,  False, (0, 180), 0.0),
        'femur': (5,  False, (0, 180), 0.0),
        'tibia': (6,  False, (0, 180), 0.0),
    },
    'lr': {
        'coxa':  (7,  True,  (0, 180), 0.0),
        'femur': (8,  True,  (0, 180), 0.0),
        'tibia': (9,  True,  (0, 180), 0.0),
    },
    'rr': {
        'coxa':  (10, False, (0, 180), 0.0),
        'femur': (11, False, (0, 180), 0.0),
        'tibia': (12, False, (0, 180), 0.0),
    },
}

LEG_PHASE_OFFSET = {
    'lf': 0.0,
    'rf': 0.5,
    'lr': 0.5,
    'rr': 0.0,
}