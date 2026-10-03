"""Obsługa pada PS4. Diagnostyka: python -m robot.joystick."""
import math
from .config import CONTROLLER_DEADZONE

LEFT_STICK_X_AXIS = 0
LEFT_STICK_Y_AXIS = 1


def open_controller(joystick_index=0):
    import pygame
    pygame.init()
    pygame.joystick.init()
    try:
        if not 0 <= joystick_index < pygame.joystick.get_count():
            raise RuntimeError("Nie wykryto kontrolera. Sprawdź połączenie pada PS4.")
        controller = pygame.joystick.Joystick(joystick_index)
        controller.init()
        print(f"Podłączono kontroler: {controller.get_name()}")
        return controller
    except Exception:
        pygame.quit()
        raise


def apply_deadzone(value, deadzone=CONTROLLER_DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def get_left_stick(controller, deadzone=CONTROLLER_DEADZONE):
    """Osie pada: X w prawo, Y do przodu; obie po strefie martwej."""
    import pygame
    pygame.event.pump()
    return (apply_deadzone(controller.get_axis(LEFT_STICK_X_AXIS), deadzone),
            apply_deadzone(-controller.get_axis(LEFT_STICK_Y_AXIS), deadzone))


def close_controller(controller):
    import pygame
    try:
        controller.quit()
    finally:
        pygame.quit()


def main():
    import pygame
    import time
    controller = open_controller()
    try:
        while True:
            pygame.event.pump()
            axes = [round(controller.get_axis(i), 3)
                    for i in range(controller.get_numaxes())]
            print(f"\rosie: {axes}", end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nKoniec.")
    finally:
        close_controller(controller)


if __name__ == "__main__":
    main()
