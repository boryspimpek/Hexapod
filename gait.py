import math

step_length = 40.0  # Długość kroku w mm
step_height = 20.0  # Wysokość kroku w mm
neutral_pos = (0.0, 70.0, -50.0)  # Pozycja neutralna stopy w mm

def calculate_hexapod_foot_trajectory(global_phase, step_length, step_height, neutral_pos, is_inverted=False):
    # global_phase: postęp cyklu od 0.0 do 1.0 dla całego kroku robota
    # neutral_pos: tuple/wektor (x, y, z) domyślnej pozycji stopy
    
    swing_ratio = 0.5  # W choidzie tripod zazwyczaj 50% czasu to swing, 50% stance
    
    # Wyznaczenie P_start i P_end na podstawie długości kroku (L)
    # Zakładamy ruch w osi X lokalnego układu nogi
    x_offset = step_length / 2.0
    
    p_start = [neutral_pos[0] - x_offset, neutral_pos[1], neutral_pos[2]]
    p_end   = [neutral_pos[0] + x_offset, neutral_pos[1], neutral_pos[2]]
    
    # Jeśli noga jest po drugiej stronie robota, kierunek może wymagać odwrócenia
    if is_inverted:
        p_start, p_end = p_end, p_start

    if global_phase < swing_ratio:
        # --- FAZA PRZENOSZENIA (SWING): ruch po łuku w powietrzu ---
        t = global_phase / swing_ratio  # skalowanie do [0, 1]
        
        # Liniowa interpolacja pozycji X i Y od p_start do p_end
        pos_x = p_start[0] + (p_end[0] - p_start[0]) * t
        pos_y = p_start[1] + (p_end[1] - p_start[1]) * t
        
        # Ruch pionowy (Z): parabola lub sinusoida osiągająca step_height
        pos_z = neutral_pos[2] + step_height * math.sin(t * math.pi)
        
    else:
        # --- FAZA PODPARCIA (STANCE): ruch po ziemi w przeciwnym kierunku ---
        t = (global_phase - swing_ratio) / (1.0 - swing_ratio) # skalowanie do [0, 1]
        
        # Ruch od p_end z powrotem do p_start
        pos_x = p_end[0] + (p_start[0] - p_end[0]) * t
        pos_y = p_end[1] + (p_start[1] - p_end[1]) * t
        
        # Zostajemy na poziomie ziemi
        pos_z = neutral_pos[2]
        
    return (pos_x, pos_y, pos_z)


# Liczba kroków/próbek w całym cyklu (np. 10 punktów)
num_samples = 10

print("Faza cyklu | X (mm)  | Y (mm)  | Z (mm)")
print("-" * 35)

for i in range(num_samples + 1):
    # Obliczamy fazę od 0.0 do 1.0
    phase = i / float(num_samples)
    
    # Wywołujemy Twoją funkcję
    x, y, z = calculate_hexapod_foot_trajectory(
        global_phase=phase, 
        step_length=step_length, 
        step_height=step_height, 
        neutral_pos=neutral_pos, 
        is_inverted=False
    )
    
    # Wypisujemy wynik sformatowany do 1 miejsca po przecinku
    print(f"{phase:10.2f} | {x:7.1f} | {y:7.1f} | {z:7.1f}")