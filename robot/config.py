# Configuration file for quadruped robot parameters

l_coxa, l_femur, l_tibia = 43.73, 100.0, 149.10

gait_speed = 1  # ile "cykli chodu" na sekundę — to jest "prędkość"
step_length = 40.0  # długość kroku w mm
step_height = 40.0  # wysokość unoszenia stopy w mm
ramp_time = 0.5  # czas narastania/zanikania prędkości chodu w sekundach
stick_deadzone = 0.1  # strefa martwa joystika

# Pozycja startowa stopy [mm] (x, y, z) w układzie współrzędnych danej nogi

z_height = -40
x_offset_front = 30
y_offset_front = 110
x_offset_rear = -30
y_offset_rear = 110
p_start = {
    'lf': (x_offset_front, y_offset_front, z_height),
    'rf': (x_offset_front, y_offset_front, z_height),
    'lr': (x_offset_rear, y_offset_rear, z_height),
    'rr': (x_offset_rear, y_offset_rear, z_height),
}

# Indeksy pól w krotce (servo_id, inverted, limits, trim)
SERVO_ID, INVERTED, LIMITS, TRIM = 0, 1, 2, 3

LEGS = {
    'lf': {
        'coxa':  (1,  True,  (50, 110), 0.0),
        'femur': (2,  False,  (90, 180), 0.0),
        'tibia': (3,  False,  (0, 130), 0.0),
    },
    'rf': {
        'coxa':  (4,  False, (70, 130), 0.0),
        'femur': (5,  True, (0, 90), 0.0),
        'tibia': (6,  True, (50, 180), 0.0),
    },
    'lr': {
        'coxa':  (7,  True,  (70, 130), 0.0),
        'femur': (8,  False,  (90, 180), 0.0),
        'tibia': (9,  False,  (0, 130), 0.0),
    },
    'rr': {
        'coxa':  (10,  False, (50, 110), 0.0),
        'femur': (11,  True, (0, 90), 0.0),
        'tibia': (12,  True, (50, 180), 0.0),
    },
}

LEG_PHASE_OFFSET = {
    'lf': 0.0,
    'rf': 0.5,
    'lr': 0.5,
    'rr': 0.0,
}

# Pozycje początkowe nóg (punkt coxa) w układzie współrzędnych robota (x, y, z) w mm 
# used for visualization
LEG_ORIGINS = {
    "lf": (80, 30, 0),
    "rf": (80, -30, 0),
    "lr": (0, 30, 0),
    "rr": (0, -30, 0),
}
# Komunikacja i wybór aktywnych serw.
ESP = ("192.168.0.115", 8888)
ACTIVE_SERVO_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}
LOOP_INTERVAL = 0.02
CONTROLLER_DEADZONE = 0.15
HEIGHT_STEP = 5.0  # mm na nacisniecie D-pada
HEIGHT_LIMITS = (-150.0, -20.0)  # zakres z_height podczas sterowania
# D-pad jako przyciski (fallback, gdy sterownik nie udostepnia hat).
DPAD_UP_BUTTON = 11
DPAD_DOWN_BUTTON = 12

# SERVO OFFSETS due to mounting and mechanical design. These offsets are applied 
# to the raw angles calculated by the kinematics to get the actual servo command angles.
COXA_ZERO = 90.0 # specjalne przesunięcie dla cox, aby 0 stopni było wzdłuż osi robota
TIBIA_OFFSET = -25.0 # specjalne ofset poniewaź odcinek tibia jest zakręcony po łuku
