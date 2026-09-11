"""
Moduł obsługujący kontroler PS4 DualShock (przez pygame) i udostępniający
odczyt lewego drążka analogowego jako wartości (jx, jy) w zakresie [-1, 1].

Wymagania:
    pip install pygame

Uwaga (Linux): kontroler musi być sparowany przez Bluetooth (lub podłączony
USB) ZANIM uruchomisz skrypt — pygame korzysta z SDL, które widzi go jako
zwykły joystick, niezależnie od tego jaki to model.

Sprawdzone mapowanie osi dla DualShock 4 pod Linuksem/Windowsem (SDL2):
    oś 0 -> lewy drążek, X (lewo -1.0 / prawo +1.0)
    oś 1 -> lewy drążek, Y (góra -1.0 / dół +1.0)  <- odwrócone względem
                                                       intuicyjnego "przód"
Jeśli Twój pad zwraca inne wartości (np. po podłączeniu przez inny sterownik),
uruchom ten plik bezpośrednio (python pad_controller.py) — wypisze surowe
wartości wszystkich osi, żebyś mógł poprawić stałe poniżej.
"""

import pygame

LEFT_STICK_X_AXIS = 0
LEFT_STICK_Y_AXIS = 1

DEFAULT_DEADZONE = 0.15


class PS4Controller:
    def __init__(self, deadzone: float = DEFAULT_DEADZONE, joystick_index: int = 0):
        pygame.init()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError(
                "Nie wykryto żadnego kontrolera. Sprawdź, czy pad PS4 jest "
                "sparowany (Bluetooth) lub podłączony (USB)."
            )

        self.joystick = pygame.joystick.Joystick(joystick_index)
        self.joystick.init()
        self.deadzone = deadzone

        print(f"Podłączono kontroler: {self.joystick.get_name()}")

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        # przeskalowanie tak, żeby zaraz za strefą martwą wartość zaczynała
        # się płynnie od 0, a nie skokiem
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def get_left_stick(self):
        """Zwraca (stick_x, stick_y) w zakresie [-1, 1] po zastosowaniu
        strefy martwej. To jest układ WSPÓŁRZĘDNYCH PADA, nie robota:

        stick_x: lewo(-) / prawo(+)
        stick_y: dodatnie = wychylenie "do przodu" (oś Y jest odwrócona
            względem surowego SDL, gdzie wychylenie w dół drążka daje +1)

        Zamiana na układ robota (X=przód/tył, Y=boki, zgodnie z konwencją
        typu ROS REP-103) odbywa się poza tą klasą — patrz komentarz w
        miejscu wywołania (np. gait_trajectory_live.py).
        """
        pygame.event.pump()  # konieczne, aby pygame odświeżył stan osi

        raw_x = self.joystick.get_axis(LEFT_STICK_X_AXIS)
        raw_y = self.joystick.get_axis(LEFT_STICK_Y_AXIS)

        stick_x = self._apply_deadzone(raw_x)
        stick_y = self._apply_deadzone(-raw_y)

        return stick_x, stick_y

    def close(self):
        self.joystick.quit()
        pygame.joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    # Tryb diagnostyczny: pokazuje surowe wartości wszystkich osi,
    # przydatne gdy Twój pad ma inne mapowanie niż zakładane wyżej.
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Brak wykrytego kontrolera.")
    else:
        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"Kontroler: {js.get_name()}, liczba osi: {js.get_numaxes()}")
        print("Ctrl+C aby przerwać.\n")
        try:
            while True:
                pygame.event.pump()
                axes = [round(js.get_axis(i), 3) for i in range(js.get_numaxes())]
                print(f"\rosie: {axes}", end="")
        except KeyboardInterrupt:
            print("\nKoniec.")