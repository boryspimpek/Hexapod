"""
Kinematyka odwrotna (IK) dla 3-stopniowej nogi quadrupeda (Coxa, Femur, Tibia).

Konwencja osi (WAŻNE):
    - X: kierunek przód/tył robota -> zmiana X obraca staw Coxa (sweep)
    - Y: kierunek na zewnątrz nogi (w stronę spoczynkowego rozstawienia)
         -> zmiana Y zmienia wysięg nogi (r), pracują Femur/Tibia
    - Z: pionowo w górę/w dół

    W spoczynku (theta_coxa = 0) noga jest skierowana wzdłuż osi +Y.

Struktura modułu (same funkcje, bez klas):
    - calculate_leg_ik   -> czyste obliczenia IK (kąty stawów)
    - calculate_joints   -> pozycje 3D punktów nogi na podstawie kątów
"""

import math

def calculate_leg_ik(x, y, z, l_coxa, l_femur, l_tibia):
    """
    Oblicza kąty (w stopniach) stawów Coxa, Femur, Tibia dla zadanej
    pozycji stopy (x, y, z) względem stawu Coxa.

    Uwaga na konwencję osi: X = przód/tył (obrót coxa), Y = wysięg nogi,
    Z = pion. Patrz docstring modułu.

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

    # Funkcja zwraca kąty w stopniach, gotowe do użycia w serwach. Uwzględnia kierunek i sposób montażu.

    return (
        #### COXA: ####
        # IK zwraca kąty np + 20, -20 w lewo i w prawo od osi y, dodajemy 90 stopni, 
        # aby kąt był liczony od zera, a nie od osi y, 
        # ponieważ takiich wartości spodziewają się serwa
        90 + math.degrees(theta_coxa_rad), 
        #### FEMUR: ###
        # IK zwraca gotowy kąt dla serwa ponieważ mamy alfa + beta
        math.degrees(theta_femur_rad),
        #### TIBIA: ###
        # Serwo jest zamontowane orczykiem i obraca sie w przeciwną stonę niż obliczony kąt, więc odejmujemy od 180 stopni
        180 - math.degrees(theta_tibia_rad),
    )


def calculate_joints(x, y, z, l_coxa, l_femur, l_tibia):
    """
    Oblicza pozycje 3D punktów: podstawa, staw biodrowy, kolano, stopa.
    Jeśli punkt jest poza zasięgiem, kąty są zerowe (noga "złożona"),
    a informacja o błędzie jest zwracana jako trzeci element krotki.
    """
    try:
        c, f, t = calculate_leg_ik(x, y, z, l_coxa, l_femur, l_tibia)
        error_msg = ""
    except ValueError as e:
        error_msg = str(e)
        c, f, t = 0.0, 0.0, 0.0

    c_rad = math.radians(c-90) # korekta o 90 stopni ponieważ na wykresie ustawiam kąt od środka
    f_rad = math.radians(f-90) # korekta o 90 stopni ponieważ na wykresie ustawiam kąt od poziomu, a obliczony jest od pionu

    p0 = (0.0, 0.0, 0.0)
    p_hip = (l_coxa * math.sin(c_rad), l_coxa * math.cos(c_rad), 0.0)
    p_knee = (
        p_hip[0] + l_femur * math.cos(f_rad) * math.sin(c_rad),
        p_hip[1] + l_femur * math.cos(f_rad) * math.cos(c_rad),
        p_hip[2] + l_femur * math.sin(f_rad),
    )
    p_foot = (x, y, z)

    points = (p0, p_hip, p_knee, p_foot)
    angles = (c, f, t)
    return points, angles, error_msg