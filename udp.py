import socket, json

ESP = ("192.168.0.115", 8888)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

print("Podaj pary: id kąt, np. '10 90' lub '10 90 11 45 12 120'. Ctrl+C kończy.")
while True:
    parts = input("> ").split()
    if len(parts) < 2 or len(parts) % 2:
        print("Podaj pary id kąt")
        continue
    cmd = {"set_servo": {parts[i]: float(parts[i + 1]) for i in range(0, len(parts), 2)}}
    sock.sendto(json.dumps(cmd).encode(), ESP)
    try:
        print("ESP:", sock.recvfrom(64)[0].decode())
    except socket.timeout:
        print("brak odpowiedzi")