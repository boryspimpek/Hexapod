import pygame
import time

# Inicjalizacja modułu joysticka w pygame
pygame.init()
pygame.joystick.init()

# Sprawdzenie, czy podłączono jakikolwiek kontroler
if pygame.joystick.get_count() == 0:
    print("Nie wykryto żadnego kontrolera! Podłącz pad PS4.")
    exit()

# Pobranie pierwszego dostępnego kontrolera
controller = pygame.joystick.Joystick(0)
controller.init()

print(f"Połączono z kontrolerem: {controller.get_name()}")
print(f"Liczba osi (analogi/triggers): {controller.get_numaxes()}")
print(f"Liczba przycisków: {controller.get_numbuttons()}")
print("-" * 50)

try:
    while True:
        # Odświeżanie zdarzeń pygame
        pygame.event.pump()

        # Odczyt osi (dla PS4: 0 = Lewy X, 1 = Lewy Y, 2 = Prawy X, 3 = Prawy Y, 4 = L2, 5 = R2)
        left_x = controller.get_axis(0)
        left_y = controller.get_axis(1)
        right_x = controller.get_axis(2)
        right_y = controller.get_axis(3)
        
        # Wyczyszczenie konsoli (opcjonalnie, żeby wyświetlało się w tym samym miejscu)
        # print("\033[H\033[J", end="") 

        print(f"Lewy X: {left_x:5.2f} | Lewy Y: {left_y:5.2f} || Prawy X: {right_x:5.2f} | Prawy Y: {right_y:5.2f}", end="\r")

        # Przykład odczytu konkretnego przycisku (np. X / Krzyżyk to zazwyczaj indeks 0)
        # if controller.get_button(0):
        #     print("\nNaciśnięto X!")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nZakończono działanie skryptu.")
    pygame.quit()