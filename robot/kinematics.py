"""
Kinematyka odwrotna (IK) dla 3-stopniowej nogi quadrupeda (Coxa, Femur, Tibia).

Konwencja osi (WAŻNE):
    - X: kierunek przód/tył robota -> zmiana X obraca staw Coxa (sweep)
    - Y: kierunek na zewnątrz nogi (w stronę spoczynkowego rozstawienia)
         -> zmiana Y zmienia wysięg nogi (r), pracują Femur/Tibia
    - Z: pionowo w górę/w dół
"""

import math

from .config import COXA_ZERO, TIBIA_OFFSET

def inverse_kinematics(x, y, z, l_coxa, l_femur, l_tibia):
    """
    Oblicza kąty (w stopniach) stawów Coxa, Femur, Tibia dla zadanej
    pozycji stopy (x, y, z) względem stawu Coxa.

    Zwraca:
        (theta_coxa_deg, theta_femur_deg, theta_tibia_deg)

    Rzuca:
        ValueError, jeśli punkt jest poza zasięgiem nogi.
    """
    theta_coxa_rad = math.atan2(x, y)
    r = math.hypot(x, y)
    r_prime = r - l_coxa
    R = math.hypot(r_prime, z)

    max_reach = l_femur + l_tibia
    min_reach = abs(l_femur - l_tibia)

    if R > max_reach or R < min_reach:
        raise ValueError(
            f"Punkt docelowy ({x:.1f}, {y:.1f}, {z:.1f}) poza zasięgiem! "
            f"R={R:.2f} (zakres: [{min_reach:.2f}, {max_reach:.2f}])"
        )

    cos_tibia = (l_femur**2 + l_tibia**2 - R**2) / (2 * l_femur * l_tibia)
    cos_tibia = max(-1.0, min(1.0, cos_tibia))
    theta_tibia_rad = math.acos(cos_tibia)

    alpha = math.atan2(r_prime, -z)
    cos_beta = (l_femur**2 + R**2 - l_tibia**2) / (2 * l_femur * R)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    theta_femur_rad = alpha + beta

    return (
        #### COXA: ####
        # IK zwraca kąty np + 20, -20 w lewo i w prawo od osi y, dodajemy COXA_ZERO (90 stopni),
        # aby kąt był liczony od zera, a nie od osi y, ponieważ takich wartości spodziewają się serwa
        COXA_ZERO + math.degrees(theta_coxa_rad),

        #### FEMUR: ####
        math.degrees(theta_femur_rad),

        #### TIBIA: ####
        # Tibia jest zamontowana w taki sposób, że jej kąt 0 lub 180 stopni nie jest w pełni wyprostowany, 
        # ale lekko zgięty (TIBIA_OFFSET), ofset jest dodany, aby końcówka stopy przy ustawiniu poziomym całej nogi była w lini.
        # Dodatkowo, w konwencji IK kąt Tibia jest liczony w przeciwnym kierunku niż w serwie, 
        # ponieważ serwo zamontowane jest tak, że orczyk jest sztywny, a obraca się serwo.
        180 - (math.degrees(theta_tibia_rad) - TIBIA_OFFSET))

def forward_kinematics(angles, l_coxa, l_femur, l_tibia):
    """Punkty stawów w lokalnym układzie nogi, w konwencji IK."""
    coxa = math.radians(angles[0] - COXA_ZERO)
    femur, tibia = map(math.radians, angles[1:])
    radial = (math.sin(coxa), math.cos(coxa))
    knee_direction = femur - tibia + math.radians(TIBIA_OFFSET)
    a = (l_coxa * radial[0], l_coxa * radial[1], 0)
    b = (a[0] + l_femur * math.sin(femur) * radial[0],
         a[1] + l_femur * math.sin(femur) * radial[1],
         -l_femur * math.cos(femur))
    c = (b[0] + l_tibia * math.sin(knee_direction) * radial[0],
         b[1] + l_tibia * math.sin(knee_direction) * radial[1],
         b[2] - l_tibia * math.cos(knee_direction))
    return [(0, 0, 0), a, b, c]
