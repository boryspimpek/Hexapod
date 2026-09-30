import json
import math
import socket
from config import LEGS, LIMITS, SERVO_ID

ESP = ("192.168.0.115", 8888)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)
SERVO_LIMITS = {
    joint_data[SERVO_ID]: joint_data[LIMITS]
    for joints in LEGS.values()
    for joint_data in joints.values()
}

print("Podaj pary: id kąt, np. '6 45' lub '6 45 8 120'. Wpisz 'q', aby zakończyć.")
try:
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
                if servo_id not in SERVO_LIMITS:
                    raise ValueError(f"brak serwa o numerze {servo_id} w config.py")

                minimum, maximum = SERVO_LIMITS[servo_id]
                limited_angle = max(minimum, min(maximum, angle))
                if limited_angle != angle:
                    print(
                        f"Serwo {servo_id}: kąt {angle:g}° poza limitem "
                        f"[{minimum}, {maximum}]°; wysyłam {limited_angle:g}°."
                    )
                servo_angles[str(servo_id)] = limited_angle
        except ValueError as error:
            print(f"Nieprawidłowe dane: {error}")
            continue

        cmd = {"set_servo": servo_angles}
        sock.sendto(json.dumps(cmd).encode(), ESP)
        try:
            print("ESP:", sock.recvfrom(64)[0].decode())
        except socket.timeout:
            print("brak odpowiedzi")
except (KeyboardInterrupt, EOFError):
    print("\nZakończono.")
finally:
    sock.close()