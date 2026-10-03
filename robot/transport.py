"""Transport UDP: tworzenie połączenia i wysyłanie komend ESP."""
import json
import socket
from . import config as cfg


def open_socket(timeout=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    return sock


def encode_servos(angles, precision=None):
    values = {str(sid): round(angle, precision) if precision is not None else angle
              for sid, angle in angles.items()}
    return json.dumps({"set_servo": values}).encode()


def send_servos(sock, angles, address=None, precision=None):
    if angles:
        sock.sendto(encode_servos(angles, precision), cfg.ESP if address is None else address)
