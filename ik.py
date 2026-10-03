"""
Kinematyka odwrotna (IK) dla 3-stopniowej nogi quadrupeda (Coxa, Femur, Tibia).

Konwencja osi (WAŻNE):
    - X: kierunek przód/tył robota -> zmiana X obraca staw Coxa (sweep)
    - Y: kierunek na zewnątrz nogi (w stronę spoczynkowego rozstawienia)
         -> zmiana Y zmienia wysięg nogi (r), pracują Femur/Tibia
    - Z: pionowo w górę/w dół

    W spoczynku (theta_coxa = 0) noga jest skierowana wzdłuż osi +Y.
"""

import math

from config import INVERTED

def inverse_kinematics(x, y, z, l_coxa, l_femur, l_tibia):
    """
    Oblicza kąty (w stopniach) stawów Coxa, Femur, Tibia dla zadanej
    pozycji stopy (x, y, z) względem stawu Coxa. Kąty są gotowe do użycia w serwach, 
    uwzględniając kierunek i sposób montażu.

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
        # IK zwraca kąty np + 20, -20 w lewo i w prawo od osi y, dodajemy 90 stopni, 
        # aby kąt był liczony od zera, a nie od osi y, 
        # ponieważ takich wartości spodziewają się serwa
        90 + math.degrees(theta_coxa_rad), 

        math.degrees(theta_femur_rad),

        math.degrees(theta_tibia_rad) - 25)