# Configuration file for quadruped robot parameters

l_coxa, l_femur, l_tibia = 40.0, 80.0, 121.0

gait_speed = 0.50  # ile "cykli chodu" na sekundę — to jest "prędkość"
step_length = 0.0  # długość kroku w mm
step_height = 40.0  # wysokość unoszenia stopy w mm
ramp_time = 0.5  # czas narastania/zanikania prędkości chodu w sekundach
stick_deadzone = 0.1  # strefa martwa joystika

# Pozycja startowa stopy [mm] (x, y, z) w układzie współrzędnych danej nogi

z_height = -62
x_offset_front = 101
y_offset_front = 102
x_offset_rear = -101
y_offset_rear = 102
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
        'coxa':  (1,  True,  (45, 160), 0.0),
        'femur': (2,  False,  (90, 180), -7.0),
        'tibia': (3,  False,  (0, 150), 0.0),
    },
    'rf': {
        'coxa':  (4,  False, (20, 135), 0.0),
        'femur': (5,  True, (0, 90), 3.0),
        'tibia': (6,  True, (30, 180), 0.0),
    },
    'lr': {
        'coxa':  (7,  True,  (20, 135), 0.0),
        'femur': (8,  True,  (0, 90), 5.0),
        'tibia': (9,  True,  (30, 180), 0.0),
    },
    'rr': {
        'coxa':  (10,  False, (45, 160), 0.0),
        'femur': (11,  False, (90, 180), -6.0),
        'tibia': (12,  False, (0, 150), 0.0),
    },
}

LEG_PHASE_OFFSET = {
    'lf': 0.0,
    'rf': 0.5,
    'lr': 0.5,
    'rr': 0.0,
}

# Pozycje początkowe nóg (punkt coxa) w układzie współrzędnych robota (x, y, z) w mm 
# Used for visualization and turning geometry (center and foot radii).
LEG_ORIGINS = {
    "lf": (80, 30, 0),
    "rf": (80, -30, 0),
    "lr": (0, 30, 0),
    "rr": (0, -30, 0),
}
# Obrót mocowania coxa od lokalnej osi +Y (na zewnątrz) w stronę +X.
# Przód: +45° do przodu; tył: -45° do tyłu, tak samo po obu stronach.
COXA_MOUNT_ANGLES = {"lf": 45.0, "rf": 45.0, "lr": -45.0, "rr": -45.0}

# Komunikacja i wybór aktywnych serw.
ESP = ("192.168.0.115", 8888)
ACTIVE_SERVO_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}
LOOP_INTERVAL = 0.02
CONTROLLER_DEADZONE = 0.15
LEFT_STICK_X_AXIS = 0
LEFT_STICK_Y_AXIS = 1
RIGHT_STICK_X_AXIS = 2
RIGHT_STICK_Y_AXIS = 3
BODY_SHIFT_MAX = 20.0  # mm per axis at full right-stick deflection
BODY_TILT_MAX = 8.0  # degrees per axis at full left-stick deflection with R1
turn_step_angle = 10.0  # degrees of yaw per gait cycle at full stick deflection
HEIGHT_STEP = 5.0  # mm na nacisniecie D-pada
HEIGHT_LIMITS = (-150.0, -20.0)  # zakres z_height podczas sterowania
# D-pad jako przyciski (fallback, gdy sterownik nie udostepnia hat).
DPAD_UP_BUTTON = 11
DPAD_DOWN_BUTTON = 12
R1_BUTTON = 10  # pygame index confirmed on this controller; browser uses button 5

# SERVO OFFSETS due to mounting and mechanical design. These offsets are applied 
# to the raw angles calculated by the kinematics to get the actual servo command angles.
COXA_ZERO = 90.0 # specjalne przesunięcie dla cox, aby 0 stopni było wzdłuż osi robota
TIBIA_OFFSET = -25.0 # specjalne ofset poniewaź odcinek tibia jest zakręcony po łuku
