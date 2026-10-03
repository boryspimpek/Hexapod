import math
import socket
from robot.config import LIMITS
from robot.servos import JOINTS, limit_angle
from robot.transport import open_socket, send_servos


def run(sock):
    print("Podaj pary: id kąt, np. '6 45' lub '6 45 8 120'. Wpisz 'q', aby zakończyć.")
    while True:
        parts = input("> ").split()
        if len(parts) == 1 and parts[0].lower() in {"q", "quit", "exit"}:
            break
        if len(parts) < 2 or len(parts) % 2:
            print("Podaj pary: numer_serwa kąt, np. '6 45'")
            continue

        try:
            servo_angles = {}
            for i in range(0, len(parts), 2):
                servo_id = int(parts[i])
                angle = float(parts[i + 1])
                if not math.isfinite(angle):
                    raise ValueError("kąt musi być skończoną liczbą")
                if servo_id not in JOINTS:
                    raise ValueError(f"brak serwa o numerze {servo_id} w robot/config.py")

                minimum, maximum = JOINTS[servo_id][LIMITS]
                limited_angle = limit_angle(servo_id, angle)
                if limited_angle != angle:
                    print(
                        f"Serwo {servo_id}: kąt {angle:g}° poza limitem "
                        f"[{minimum}, {maximum}]°; wysyłam {limited_angle:g}°."
                    )
                servo_angles[str(servo_id)] = limited_angle
        except ValueError as error:
            print(f"Nieprawidłowe dane: {error}")
            continue

        send_servos(sock, servo_angles)
        try:
            print("ESP:", sock.recvfrom(64)[0].decode())
        except socket.timeout:
            print("brak odpowiedzi")


def main():
    try:
        with open_socket(timeout=1.0) as sock:
            run(sock)
    except (KeyboardInterrupt, EOFError):
        print("\nZakończono.")


if __name__ == "__main__":
    main()
