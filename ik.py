import math

# Przykładowe długości segmentów (np. w milimetrach)
L_COXA = 50.0
L_FEMUR = 100.0
L_TIBIA = 100.0


def calculate_leg_ik(x, y, z, l_coxa, l_femur, l_tibia):
    """
    Oblicza kąty dla 3-stopniowej nogi hexapoda (Coxa, Femur, Tibia).
    
    Parametry:
    x, y, z   - docelowe współrzędne stopy względem stawu Coxa [mm lub cm]
    l_coxa    - długość segmentu biodrowego (L1)
    l_femur   - długość segmentu udowego (L2)
    l_tibia   - długość segmentu goleniowego (L3)
    
    Zwraca:
    tuple (theta_coxa, theta_femur, theta_tibia) w stopniach
    """
    
    # 1. Kąt Coxa (obrót w poziomie wokół osi Z)
    theta_coxa_rad = math.atan2(y, x)
    
    # Odległość rzutu stopy na płaszczyznę XY od osi obrotu coxa
    r = math.sqrt(x**2 + y**2)
    
    # Efektywny zasięg w płaszczyźnie pionowej nogi (po odjęciu długości coxa)
    r_prime = r - l_coxa
    
    # Odległość wprost z osi stawu femur do stopy w trójkącie (r_prime, z)
    # Zakładamy, że z jest dodatnie w dół (lub używamy wartości bezwzględnej/odpowiedniego znaku)
    R = math.sqrt(r_prime**2 + z**2)
    
    # Sprawdzenie ograniczeń fizycznych (zasięg maksymalny i minimalny)
    max_reach = l_femur + l_tibia
    min_reach = abs(l_femur - l_tibia)
    
    if R > max_reach or R < min_reach:
        raise ValueError(f"Punkt docelowy ({x}, {y}, {z}) jest poza zasięgiem nogi! "
                         f"Odległość R={R:.2f}, dozwolony zakres: [{min_reach:.2f}, {max_reach:.2f}]")
    
    # 2. Kąt Tibia (kolano) z twierdzenia kosinusów
    # R^2 = l_femur^2 + l_tibia^2 - 2 * l_femur * l_tibia * cos(pi - theta_tibia)
    cos_tibia = (l_femur**2 + l_tibia**2 - R**2) / (2 * l_femur * l_tibia)
    # Zabezpieczenie przed błędami zmiennoprzecinkowymi (np. wartości 1.0000001)
    cos_tibia = max(-1.0, min(1.0, cos_tibia))
    
    # Kąt zgięcia kolana (w zależności od konstrukcji: kolano w górę / w dół)
    # Poniższy wzór zakłada kolano skierowane do góry
    theta_tibia_rad = math.acos(cos_tibia)
    
    # 3. Kąt Femur (udo)
    # Kąt linii łączącej staw femur ze stopą względem poziomu
    alpha = math.atan2(z, r_prime)
    
    # Kąt wewnątrz trójkąta przy stawie femur
    cos_beta = (l_femur**2 + R**2 - l_tibia**2) / (2 * l_femur * R)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    
    # Suma kątów daje ostateczny kąt serwa femur
    theta_femur_rad = alpha + beta
    
    # Konwersja wyników na stopnie
    theta_coxa = math.degrees(theta_coxa_rad)
    theta_femur = math.degrees(theta_femur_rad)
    theta_tibia = math.degrees(theta_tibia_rad)
    
    return theta_coxa, theta_femur, theta_tibia

# --- Przykład użycia ---
if __name__ == "__main__":
    # Docelowa pozycja stopy względem stawu biodrowego tej konkretnej nogi
    target_x = 120.0
    target_y = 50.0
    target_z = 80.0  # Pozycja pod poziomem biodra
    
    try:
        c, f, t = calculate_leg_ik(target_x, target_y, target_z, L_COXA, L_FEMUR, L_TIBIA)
        print(f"Obliczone kąty dla nogi:")
        print(f"Coxa (biodro):  {c:.2f}°")
        print(f"Femur (udo):    {f:.2f}°")
        print(f"Tibia (goleń):  {t:.2f}°")
    except ValueError as e:
        print(f"Błąd IK: {e}")