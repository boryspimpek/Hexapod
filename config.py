# Configuration file for quadruped robot parameters

l_coxa, l_femur, l_tibia = 43.73, 100.0, 149.10

gait_speed = 0.5  # ile "cykli chodu" na sekundę — to jest "prędkość"
step_length = 40.0  # długość kroku w mm
step_height = 40.0  # wysokość unoszenia stopy w mm
ramp_time = 0.5  # czas narastania/zanikania prędkości chodu w sekundach
stick_deadzone = 0.1  # strefa martwa joystika

# Pozycja startowa stopy [mm] (x, y, z) w układzie współrzędnych danej nogi
p_start = {
    'lf': (0, 165, -40),
    'rf': (0, 165, -40),
    'lr': (0, 165, -40),
    'rr': (0, 165, -40),
}

# Indeksy pól w krotce (servo_id, inverted, limits, trim)
SERVO_ID, INVERTED, LIMITS, TRIM = 0, 1, 2, 3

LEGS = {
    'lf': {
        'coxa':  (1,  False,  (50, 110), 0.0),
        'femur': (2,  False,  (90, 180), 0.0),
        'tibia': (3,  False,  (0, 130), 0.0),
    },
    'rf': {
        'coxa':  (4,  True, (70, 130), 0.0),
        'femur': (5,  True, (0, 90), 0.0),
        'tibia': (6,  True, (50, 180), 0.0),
    },
    'lr': {
        'coxa':  (7,  False,  (50, 110), 0.0),
        'femur': (8,  False,  (90, 180), 0.0),
        'tibia': (9,  False,  (0, 130), 0.0),
    },
    'rr': {
        'coxa':  (10,  True, (70, 130), 0.0),
        'femur': (11,  True, (0, 90), 0.0),
        'tibia': (12,  True, (50, 180), 0.0),
    },
}

LEG_PHASE_OFFSET = {
    'lf': 0.5,
    'rf': 0.0,
    'lr': 0.5,
    'rr': 0.0,
}

LEG_ORIGINS = {
    "lf": (80, -30, 0),
    "rf": (80, 30, 0),
    "lr": (0, -30, 0),
    "rr": (0, 30, 0),
}