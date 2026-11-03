# Konfiguracja robota czworonożnego.

# --- Geometria nóg i mocowań ---

# Długości segmentów nogi [mm].
l_coxa, l_femur, l_tibia = 40.0, 80.0, 121.0

# Współrzędne (x, y, z) [mm] w lokalnym układzie danej nogi.
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

# --- Parametry kroku i rodzaje chodu ---

trot_gait_speed = 1.50  # liczba cykli chodu na sekundę
crawl_gait_speed = 0.25  # liczba cykli chodu na sekundę

step_length = 120.0  # maksymalna długość kroku [mm]
MIN_STEP_LENGTH = 40.0  # minimalna długość kroku przy aktywnym ruchu [mm]

turn_step_angle = 30.0  # obrót na cykl chodu przy pełnym wychyleniu drążka [°]
MIN_TURN_STEP_ANGLE = 10.0  # minimalny obrót na cykl chodu przy aktywnym ruchu [°]

step_height = 40.0  # wysokość unoszenia stopy [mm]

ramp_time = 0.5  # czas narastania/zanikania prędkości chodu [s]
stick_deadzone = 0.1  # strefa martwa joysticka

# --- Sterowanie pozycją korpusu ---

BODY_SHIFT_MAX = 30.0  # przesunięcie na oś przy pełnym wychyleniu prawego drążka [mm]
BODY_TILT_MAX = 8.0  # przechył na oś przy pełnym wychyleniu lewego drążka z R1 [°]
HEIGHT_STEP = 5.0  # zmiana wysokości na naciśnięcie D-pada [mm]
HEIGHT_LIMITS = (-150.0, -20.0)  # zakres z_height podczas sterowania [mm]


# Punkty mocowania coxa w układzie robota (x, y, z) [mm].
# Używane do wizualizacji i geometrii skrętu (środek i promienie stóp).
LEG_ORIGINS = {
    "lf": (80, 30, 0),
    "rf": (80, -30, 0),
    "lr": (0, 30, 0),
    "rr": (0, -30, 0),
}

# Obrót mocowania coxa od lokalnej osi +Y (na zewnątrz) w stronę +X.
# Przód: +45° do przodu; tył: -45° do tyłu, tak samo po obu stronach.
COXA_MOUNT_ANGLES = {
    "lf": 45.0,
    "rf": 45.0,
    "lr": -45.0,
    "rr": -45.0,
}

# Przesunięcia wynikające z montażu i mechaniki, dodawane do kątów z kinematyki.
COXA_ZERO = 90.0  # przesunięcie coxa, aby 0° było wzdłuż osi robota
TIBIA_OFFSET = -25.0  # korekta uwzględniająca wygięcie segmentu tibia

# --- Pozycje startowe stóp ---


# --- Serwa i kalibracja ---

# Indeksy pól w krotce (servo_id, inverted, limits, trim).
SERVO_ID, INVERTED, LIMITS, TRIM = 0, 1, 2, 3

LEGS = {
    'lf': {
        'coxa':  (1,  True,  (45, 160),  0.0),
        'femur': (2,  False, (90, 180), -7.0),
        'tibia': (3,  False, (0, 150),   0.0),
    },
    'rf': {
        'coxa':  (4,  False, (20, 135),  0.0),
        'femur': (5,  True,  (0, 90),    3.0),
        'tibia': (6,  True,  (30, 180),  0.0),
    },
    'lr': {
        'coxa':  (7,  True,  (20, 135),  0.0),
        'femur': (8,  True,  (0, 90),    5.0),
        'tibia': (9,  True,  (30, 180),  0.0),
    },
    'rr': {
        'coxa':  (10, False, (45, 160),  0.0),
        'femur': (11, False, (90, 180), -6.0),
        'tibia': (12, False, (0, 150),   0.0),
    },
}

ACTIVE_SERVO_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}

# Przesunięcia faz nóg dla chodu trot.
LEG_PHASE_OFFSET = {
    'lf': 0.0,
    'rf': 0.5,
    'lr': 0.5,
    'rr': 0.0,
}

# Wybór chodu i część cyklu spędzona z nogą w powietrzu.
GAITS = {
    "trot": {"swing_ratio": 0.5, "phase_offsets": LEG_PHASE_OFFSET},
    "crawl": {
        "swing_ratio": 0.25,
        # Przesunięcie korpusu przeciwnie do unoszonej nogi [mm]:
        # (przód/tył, lewo/prawo). (0.0, 0.0) wyłącza efekt.
        "body_shift": (20.0, 20.0),
        # Kolejność: lf -> lr -> rf -> rr (faza lokalna = faza + offset).
        "phase_offsets": {"lf": 0.0, "lr": 0.25, "rf": 0.5, "rr": 0.75},
    },
}
DEFAULT_GAIT = "trot"
GAIT_CYCLE = ("trot", "crawl")


# --- Komunikacja i pętla sterowania ---

ESP = ("192.168.0.115", 8888)
LOOP_INTERVAL = 0.02  # odstęp między iteracjami pętli [s]


# --- Kontroler: osie i przyciski ---

CONTROLLER_DEADZONE = 0.15

LEFT_STICK_X_AXIS = 0
LEFT_STICK_Y_AXIS = 1
RIGHT_STICK_X_AXIS = 2
RIGHT_STICK_Y_AXIS = 3

GAIT_BUTTON = 0  # X / krzyżyk PS4; sprawdź indeks: python -m robot.joystick
R1_BUTTON = 10  # indeks pygame na tym kontrolerze; w przeglądarce: przycisk 5

# D-pad jako przyciski (wariant zapasowy, gdy sterownik nie udostępnia hat).
DPAD_UP_BUTTON = 11
DPAD_DOWN_BUTTON = 12


