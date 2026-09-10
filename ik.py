import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Przykładowe długości segmentów (np. w milimetrach)
L_COXA = 50.0
L_FEMUR = 80.0
L_TIBIA = 120.0


def calculate_leg_ik(x, y, z, l_coxa, l_femur, l_tibia):
    """
    Oblicza kąty dla 3-stopniowej nogi hexapoda (Coxa, Femur, Tibia).
    """
    theta_coxa_rad = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)
    r_prime = r - l_coxa
    R = math.sqrt(r_prime**2 + z**2)
    
    max_reach = l_femur + l_tibia
    min_reach = abs(l_femur - l_tibia)
    
    if R > max_reach or R < min_reach:
        raise ValueError(f"Punkt docelowy ({x}, {y}, {z}) jest poza zasięgiem nogi! "
                         f"Odległość R={R:.2f}, dozwolony zakres: [{min_reach:.2f}, {max_reach:.2f}]")
    
    cos_tibia = (l_femur**2 + l_tibia**2 - R**2) / (2 * l_femur * l_tibia)
    cos_tibia = max(-1.0, min(1.0, cos_tibia))
    theta_tibia_rad = math.acos(cos_tibia)
    
    alpha = math.atan2(z, r_prime)
    cos_beta = (l_femur**2 + R**2 - l_tibia**2) / (2 * l_femur * R)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    
    theta_femur_rad = alpha + beta
    
    theta_coxa = math.degrees(theta_coxa_rad)
    theta_femur = math.degrees(theta_femur_rad)
    theta_tibia = math.degrees(theta_tibia_rad)
    
    return theta_coxa, theta_femur, theta_tibia


def visualize_leg(x, y, z, l_coxa, l_femur, l_tibia):
    """
    Wizualizuje pozycję i geometrię 3D nogi hexapoda na podstawie docelowych współrzędnych.
    """
    try:
        c, f, t = calculate_leg_ik(x, y, z, l_coxa, l_femur, l_tibia)
    except ValueError as e:
        print(f"Błąd IK podczas wizualizacji: {e}")
        return

    c_rad = math.radians(c)
    f_rad = math.radians(f)

    # Współrzędne kluczowych punktów stawów
    p0 = (0.0, 0.0, 0.0)  # Baza (początek biodra)
    phip = (l_coxa * math.cos(c_rad), l_coxa * math.sin(c_rad), 0.0)  # Koniec coxa / początek femur
    pknee = (
        phip[0] + l_femur * math.cos(f_rad) * math.cos(c_rad),
        phip[1] + l_femur * math.cos(f_rad) * math.sin(c_rad),
        phip[2] + l_femur * math.sin(f_rad)
    )  # Kolano (koniec femur / początek tibia)
    pfoot = (x, y, z)  # Stopa

    xs = [p0[0], phip[0], pknee[0], pfoot[0]]
    ys = [p0[1], phip[1], pknee[1], pfoot[1]]
    zs = [p0[2], phip[2], pknee[2], pfoot[2]]

    # Tworzenie wykresów 3D
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Rysowanie struktury nogi (szkielet)
    ax.plot(xs, ys, zs, 'o-', linewidth=4, markersize=8, color='#1f77b4', label='Segmenty nogi')
    
    # Oznaczenia stawów
    ax.scatter([p0[0]], [p0[1]], [p0[2]], color='black', s=100, label='Coxa')
    ax.scatter([phip[0]], [phip[1]], [phip[2]], color='orange', s=80, label='Femur')
    ax.scatter([pknee[0]], [pknee[1]], [pknee[2]], color='green', s=80, label='Tibia')
    ax.scatter([pfoot[0]], [pfoot[1]], [pfoot[2]], color='red', s=100, label='Stopa')

    # Konfiguracja osi i etykiet
    ax.set_title(f"Kinematyka odwrotna nogi Hexapoda\nKąty: Coxa={c:.1f}°, Femur={f:.1f}°, Tibia={t:.1f}°")
    ax.set_xlabel('Oś X [mm]')
    ax.set_ylabel('Oś Y [mm]')
    ax.set_zlabel('Oś Z [mm]')
    
    # Ustawienie równej skalowalności osi dla zachowania proporcji geometrii
    max_range = max(l_coxa + l_femur + l_tibia, abs(x), abs(y), abs(z))
    ax.set_xlim([-max_range/2, max_range/2])
    ax.set_ylim([-max_range/2, max_range/2])
    ax.set_zlim([-max_range/2, max_range/2])

    ax.legend()
    plt.show()


if __name__ == "__main__":
    target_x = 200
    target_y = 0
    target_z = 50  # Pozycja pod poziomem biodra
    
    try:
        c, f, t = calculate_leg_ik(target_x, target_y, target_z, L_COXA, L_FEMUR, L_TIBIA)
        print("Obliczone kąty dla nogi:")
        print(f"Coxa (biodro):  {c:.2f}°")
        print(f"Femur (udo):    {f:.2f}°")
        print(f"Tibia (goleń):  {t:.2f}°")
        
        # Uruchomienie wizualizacji
        visualize_leg(target_x, target_y, target_z, L_COXA, L_FEMUR, L_TIBIA)
        
    except ValueError as e:
        print(f"Błąd IK: {e}")