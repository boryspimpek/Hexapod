# Konfiguracja robota czworonożnego.

# --- Geometria nóg i mocowań ---

# Długości segmentów nogi [mm].
l_coxa, l_femur, l_tibia = 40.0, 80.0, 121.25

# Współrzędne (x, y, z) [mm] w lokalnym układzie danej nogi.
z_height = -50
x_offset_front = 100
y_offset_front = 100
x_offset_rear = - x_offset_front
y_offset_rear = y_offset_front
p_start = {
    'lf': (x_offset_front, y_offset_front, z_height),
    'rf': (x_offset_front, y_offset_front, z_height),
    'lr': (x_offset_rear, y_offset_rear, z_height),
    'rr': (x_offset_rear, y_offset_rear, z_height),
}




################################################### --- Parametry kroku i rodzaje chodu --- ####################################################

trot_gait_speed = 1.0  # liczba cykli chodu na sekundę
crawl_gait_speed = 0.25  # liczba cykli chodu na sekundę

step_length = 120.0  # maksymalna długość kroku [mm]
MIN_STEP_LENGTH = 20.0  # minimalna długość kroku przy aktywnym ruchu [mm]

step_height = 60.0  # wysokość unoszenia stopy [mm]

turn_step_angle = 30.0  # obrót na cykl chodu przy pełnym wychyleniu drążka [°]
MIN_TURN_STEP_ANGLE = 10.0  # minimalny obrót na cykl chodu przy aktywnym ruchu [°]

BODY_SHIFT_MAX = 60.0  # przesunięcie na oś przy pełnym wychyleniu prawego drążka [mm]
BODY_TILT_MAX = 20.0  # przechył na oś przy pełnym wychyleniu lewego drążka z R1 [°]
BODY_POSE_SMOOTHING_SECONDS = 0.30  # stała czasowa wygładzania body_shift/body_tilt [s]

HEIGHT_STEP = 5.0  # zmiana wysokości na naciśnięcie D-pada [mm]
HEIGHT_LIMITS = (-110.0, 40.0)  # zakres z_height podczas sterowania [mm]

ramp_time = 0.25  # czas narastania/zanikania prędkości chodu [s]

REST_HEIGHT = 0.0  # wysokość spoczynkowa [mm]

POSE_TRANSITION_SECONDS = 1.0  # czas płynnego przejścia po REST/WALK [s]

# Symulator: stabilność statyczna (CoM = środek geometryczny korpusu).
CONTACT_TOLERANCE = 2.0  # stopa uznana za podparcie, gdy jest najwyżej tyle nad podłożem [mm]
STABILITY_WARNING_MARGIN = 10.0  # margines CoM od krawędzi poniżej którego ostrzegamy [mm]




########################### --- Przesunięcia wynikające z montażu i mechaniki, dodawane do kątów z kinematyki --- ####################################################

COXA_ZERO = 90.0  # przesunięcie coxa, aby 0° było wzdłuż osi robota
TIBIA_OFFSET = -20.0  # korekta uwzględniająca wygięcie segmentu tibia
FEMUR_OFFSET = 15.0  # korekta uwzględniająca wygięcie segmentu femur




#################################################### --- Serwa i kalibracja --- ####################################################################

# Indeksy pól w krotce (servo_id, inverted, limits, trim).
# femury: limity zwężone do wspólnego zakresu logicznego 47-183 (oryg. 40-180 / 0-140),
# zależą od trimów, po zmianie trimu przelicz.
SERVO_ID, INVERTED, LIMITS, TRIM = 0, 1, 2, 3

LEGS = {
    'lf': {
        'coxa':  (1,  True,  (35, 160),  0.0),
        'femur': (2,  False, (40, 176), -7.0),
        'tibia': (3,  False, (0, 150),   0.0),
    },
    'rf': {
        'coxa':  (4,  False, (20, 145),  0.0),
        'femur': (5,  True,  (0, 136),    3.0),
        'tibia': (6,  True,  (30, 180),  0.0),
    },
    'lr': {
        'coxa':  (7,  True,  (20, 145),  0.0),
        'femur': (8,  True,  (2, 138),    5.0),
        'tibia': (9,  True,  (30, 180),  0.0),
    },
    'rr': {
        'coxa':  (10, False, (35, 160),  0.0),
        'femur': (11, False, (41, 177), -6.0),
        'tibia': (12, False, (0, 150),   0.0),
    },
}

# Zmierzone (oryginalne) limity wszystkich serw, w przestrzeni serwa, Limity w LEGS mogą być z nich pochodną (np. femury zwężone pod trimy).
MEASURED_LIMITS = {
    1:  (35, 160),   # lf coxa
    2:  (40, 180),   # lf femur
    3:  (0, 150),    # lf tibia
    4:  (20, 145),   # rf coxa
    5:  (0, 140),    # rf femur
    6:  (30, 180),   # rf tibia
    7:  (20, 145),   # lr coxa
    8:  (0, 140),    # lr femur
    9:  (30, 180),   # lr tibia
    10: (35, 160),   # rr coxa
    11: (40, 180),   # rr femur
    12: (0, 150),    # rr tibia
}

# Limity symulatora: przestrzeń serwa, BEZ trimów, wpisane ręcznie.
# Femury: wspólny zakres logiczny 47-183 (jak na robocie po zwężeniu limitów).
# Odwrócone (5, 8) mają odbicie: 180 - 183 = -3, 180 - 47 = 133.
# Po zmianie trimu femurów przelicz je od nowa z MEASURED_LIMITS.
SIM_LIMITS = {
    1:  (35, 160),   # lf coxa
    2:  (47, 183),   # lf femur
    3:  (0, 150),    # lf tibia
    4:  (20, 145),   # rf coxa
    5:  (-3, 133),   # rf femur (odwrócony)
    6:  (30, 180),   # rf tibia
    7:  (20, 145),   # lr coxa
    8:  (-3, 133),   # lr femur (odwrócony)
    9:  (30, 180),   # lr tibia
    10: (35, 160),   # rr coxa
    11: (47, 183),   # rr femur
    12: (0, 150),    # rr tibia
}

###################################################### --- Geometria --- ####################################################################

# Punkty mocowania coxa w układzie robota (x, y, z) [mm].
# Używane do wizualizacji i geometrii skrętu (środek i promienie stóp).
LEG_ORIGINS = {
    "lf": (89, 69, 0),
    "rf": (89, -69, 0),
    "lr": (-89, 69, 0),
    "rr": (-89, -69, 0),
}

# Obrót mocowania coxa od lokalnej osi +Y (na zewnątrz) w stronę +X.
# Przód: +45° do przodu; tył: -45° do tyłu, tak samo po obu stronach.
COXA_MOUNT_ANGLES = {
    "lf": 45.0,
    "rf": 45.0,
    "lr": -45.0,
    "rr": -45.0,
}




######################################################## --- Gaits --- ##################################################################

# Przesunięcia faz nóg dla chodu trot.
LEG_PHASE_OFFSET = {
    'lf': 0.0,
    'rf': 0.5,
    'lr': 0.5,
    'rr': 0.0,
}

# Wybór chodu i część cyklu spędzona z nogą w powietrzu.
GAITS = {
    "trot": {"swing_ratio": 0.3, "phase_offsets": LEG_PHASE_OFFSET},
    "crawl": {
        "swing_ratio": 0.25,
        # Przesunięcie korpusu przeciwnie do unoszonej nogi [mm]:
        # (przód/tył, lewo/prawo). (0.0, 0.0) wyłącza efekt.
        "body_shift": (30.0, 30.0),
        "phase_offsets": {"lf": 0.0, "lr": 0.25, "rf": 0.5, "rr": 0.75},
    },
}

DEFAULT_GAIT = "trot"
GAIT_CYCLE = ("trot", "crawl")




################################################## --- Komunikacja i pętla sterowania --- ####################################################

ESP = ("192.168.0.115", 8888)
LOOP_INTERVAL = 0.02  # odstęp między iteracjami pętli [s]

ACTIVE_SERVO_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}




####################################################### --- Kontroler: osie i przyciski --- ####################################################

CONTROLLER_DEADZONE = 0.15
stick_deadzone = 0.0  # strefa martwa joysticka

LEFT_STICK_X_AXIS = 0
LEFT_STICK_Y_AXIS = 1
RIGHT_STICK_X_AXIS = 2
RIGHT_STICK_Y_AXIS = 3

GAIT_BUTTON = 0  # X / krzyżyk PS4; sprawdź indeks: python -m robot.joystick
REST_BUTTON = 1  # O / kółko PS4; sprawdź indeks: python -m robot.joystick
WALK_BUTTON = 2  # kwadrat PS4; sprawdź indeks: python -m robot.joystick
R1_BUTTON = 10  # indeks pygame na tym kontrolerze; w przeglądarce: przycisk 5

# D-pad jako przyciski (wariant zapasowy, gdy sterownik nie udostępnia hat).
DPAD_UP_BUTTON = 11
DPAD_DOWN_BUTTON = 12


