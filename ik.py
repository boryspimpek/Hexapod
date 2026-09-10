import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button

# Długości segmentów nogi hexapoda (w milimetrach)
L_COXA = 50.0
L_FEMUR = 80.0
L_TIBIA = 120.0

# Początkowy stan pozycji stopy
current_pos = {
    'x': 200.0,
    'y': 0.0,
    'z': 50.0,
    'step': 10.0  # krok zmiany w milimetrach
}

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
        raise ValueError(f"Punkt docelowy ({x:.1f}, {y:.1f}, {z:.1f}) poza zasięgiem! R={R:.2f} (zakres: [{min_reach:.2f}, {max_reach:.2f}])")
    
    cos_tibia = (l_femur**2 + l_tibia**2 - R**2) / (2 * l_femur * l_tibia)
    cos_tibia = max(-1.0, min(1.0, cos_tibia))
    theta_tibia_rad = math.acos(cos_tibia)
    
    alpha = math.atan2(z, r_prime)
    cos_beta = (l_femur**2 + R**2 - l_tibia**2) / (2 * l_femur * R)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    
    theta_femur_rad = alpha + beta
    
    return math.degrees(theta_coxa_rad), math.degrees(theta_femur_rad), math.degrees(theta_tibia_rad)

# Inicjalizacja okna i wykresu 3D
fig = plt.figure(figsize=(12, 9))
# Dostosowanie układu, aby zrobić miejsce na przyciski na dole
plt.subplots_adjust(bottom=0.25)

ax = fig.add_subplot(111, projection='3d')

def update_plot():
    """Odświeża geometrię wykresu na podstawie aktualnych współrzędnych."""
    ax.clear()
    
    x, y, z = current_pos['x'], current_pos['y'], current_pos['z']
    
    try:
        c, f, t = calculate_leg_ik(x, y, z, L_COXA, L_FEMUR, L_TIBIA)
        error_msg = ""
    except ValueError as e:
        error_msg = f"\nBŁĄD: {e}"
        c, f, t = 0.0, 0.0, 0.0

    c_rad = math.radians(c)
    f_rad = math.radians(f)

    # Współrzędne kluczowych punktów stawów
    p0 = (0.0, 0.0, 0.0)
    phip = (L_COXA * math.cos(c_rad), L_COXA * math.sin(c_rad), 0.0)
    pknee = (
        phip[0] + L_FEMUR * math.cos(f_rad) * math.cos(c_rad),
        phip[1] + L_FEMUR * math.cos(f_rad) * math.sin(c_rad),
        phip[2] + L_FEMUR * math.sin(f_rad)
    )
    pfoot = (x, y, z)

    xs = [p0[0], phip[0], pknee[0], pfoot[0]]
    ys = [p0[1], phip[1], pknee[1], pfoot[1]]
    zs = [p0[2], phip[2], pknee[2], pfoot[2]]

    # Rysowanie struktury nogi
    ax.plot(xs, ys, zs, 'o-', linewidth=4, markersize=8, color='#1f77b4', label='Segmenty nogi')
    ax.scatter([p0[0]], [p0[1]], [p0[2]], color='black', s=100, label='Coxa')
    ax.scatter([phip[0]], [phip[1]], [phip[2]], color='orange', s=80, label='Femur')
    ax.scatter([pknee[0]], [pknee[1]], [pknee[2]], color='green', s=80, label='Tibia')
    ax.scatter([pfoot[0]], [pfoot[1]], [pfoot[2]], color='red', s=100, label='Stopa')

    # Konfiguracja osi i etykiet
    ax.set_title(f"Kinematyka odwrotna nogi Hexapoda\nPoz: X={x:.1f}, Y={y:.1f}, Z={z:.1f} | Kąty: Coxa={c:.1f}°, Femur={f:.1f}°, Tibia={t:.1f}°{error_msg}", fontsize=10)
    ax.set_xlabel('Oś X [mm]')
    ax.set_ylabel('Oś Y [mm]')
    ax.set_zlabel('Oś Z [mm]')
    
    max_range = max(L_COXA + L_FEMUR + L_TIBIA, abs(x), abs(y), abs(z)) * 1.1
    ax.set_xlim([-max_range/2, max_range/2])
    ax.set_ylim([-max_range/2, max_range/2])
    ax.set_zlim([-max_range/2, max_range/2])

    ax.legend(loc='upper left')
    fig.canvas.draw_idle()

# Pierwsze narysowanie wykresu
update_plot()

# Definicje funkcji obsługi kliknięć przycisków
def change_x_plus(event):
    current_pos['x'] += current_pos['step']
    update_plot()

def change_x_minus(event):
    current_pos['x'] -= current_pos['step']
    update_plot()

def change_y_plus(event):
    current_pos['y'] += current_pos['step']
    update_plot()

def change_y_minus(event):
    current_pos['y'] -= current_pos['step']
    update_plot()

def change_z_plus(event):
    current_pos['z'] += current_pos['step']
    update_plot()

def change_z_minus(event):
    current_pos['z'] -= current_pos['step']
    update_plot()

# Tworzenie obszarów pod przyciski [left, bottom, width, height]
ax_x_plus = plt.axes([0.15, 0.12, 0.08, 0.06])
ax_x_minus = plt.axes([0.15, 0.04, 0.08, 0.06])

ax_y_plus = plt.axes([0.45, 0.12, 0.08, 0.06])
ax_y_minus = plt.axes([0.45, 0.04, 0.08, 0.06])

ax_z_plus = plt.axes([0.75, 0.12, 0.08, 0.06])
ax_z_minus = plt.axes([0.75, 0.04, 0.08, 0.06])

# Przypisanie widgetów przycisków
btn_x_plus = Button(ax_x_plus, 'X +')
btn_x_plus.on_clicked(change_x_plus)

btn_x_minus = Button(ax_x_minus, 'X -')
btn_x_minus.on_clicked(change_x_minus)

btn_y_plus = Button(ax_y_plus, 'Y +')
btn_y_plus.on_clicked(change_y_plus)

btn_y_minus = Button(ax_y_minus, 'Y -')
btn_y_minus.on_clicked(change_y_minus)

btn_z_plus = Button(ax_z_plus, 'Z +')
btn_z_plus.on_clicked(change_z_plus)

btn_z_minus = Button(ax_z_minus, 'Z -')
btn_z_minus.on_clicked(change_z_minus)

plt.show()