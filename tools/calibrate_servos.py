"""Kalibracja trimów: python -m tools.calibrate_servos."""

import math

from robot import config as cfg
from robot.motion import JOINT_NAMES, calculate_frame
from robot.servos import JOINTS, apply_offsets, limit_angle
from robot.transport import open_socket, send_servos


def starting_positions():
    """Komendy startowe oraz kąty po inwersji, przed trimem i limitami."""
    frame = calculate_frame()
    baseline = {}
    for name, leg in frame["legs"].items():
        for joint, raw in zip(JOINT_NAMES, leg["raw_angles"]):
            sid = cfg.LEGS[name][joint][cfg.SERVO_ID]
            baseline[sid] = apply_offsets(sid, raw) - JOINTS[sid][cfg.TRIM]
    return frame["servos"], baseline


def print_trims(trims):
    print("\nPełne wartości TRIM do wpisania w robot/config.py (ostatnie pole krotki):")
    for sid in sorted(trims):
        print(f"{sid}: {trims[sid]:+.6f}".rstrip("0").rstrip("."))
    print("Config nie został zmieniony. Wynik opiera się na wysłanych komendach UDP;")
    print("skrypt nie odczytuje rzeczywistej pozycji serw.")


def run(sock):
    initial, baseline = starting_positions()
    active = set(initial) & cfg.ACTIVE_SERVO_IDS
    if not active:
        raise ValueError("Brak aktywnych serw w ACTIVE_SERVO_IDS.")
    trims = {sid: data[cfg.TRIM] for sid, data in JOINTS.items()}
    send_servos(sock, {sid: initial[sid] for sid in sorted(active)})
    try:
        print("Wysłano pozycję startową z configu (IK, inwersja, trimy i limity).")
        for sid in sorted(initial):
            if sid not in active:
                print(f"{sid}: nieaktywne — zachowuję trim {trims[sid]:+g}")
                continue
            print(f"{sid}: {initial[sid]:.6f}°")
            requested = baseline[sid] + trims[sid]
            if initial[sid] != requested:
                print(f"  Limit przyciął pozycję z {requested:.6f}°.")
        print("Podaj numer i końcowy kąt, np. '12 97' lub '4 135'.")
        print("'q' / Ctrl+C kończy i wypisuje trimy. Kąty mogą mieć część dziesiętną.")
        while True:
            parts = input("> ").split()
            if len(parts) == 1 and parts[0].lower() in {"q", "quit", "exit"}:
                break
            try:
                if len(parts) != 2:
                    raise ValueError("podaj jedną parę: numer_serwa kąt")
                sid = int(parts[0])
                angle = float(parts[1])
                if sid not in active:
                    raise ValueError("serwo nie istnieje lub nie jest aktywne w configu")
                if not math.isfinite(angle):
                    raise ValueError("kąt musi być skończoną liczbą")
            except ValueError as error:
                print(f"Nieprawidłowe dane: {error}")
                continue
            command = limit_angle(sid, angle)
            if command != angle:
                print(f"Limit serwa {sid}: zamiast {angle:g}° wysyłam {command:g}°.")
            # Wpisany kąt jest już komendą serwa: bez kolejnej inwersji i trimu.
            send_servos(sock, {sid: command})
            trims[sid] = command - baseline[sid]
            print(f"{sid}: {command:g}°, TRIM do configu: {trims[sid]:+g}")
    except (KeyboardInterrupt, EOFError):
        print("\nZakończono kalibrację.")
    finally:
        print_trims(trims)


def main():
    try:
        with open_socket() as sock:
            run(sock)
    except (OSError, ValueError) as error:
        raise SystemExit(f"Błąd kalibracji: {error}") from error


if __name__ == "__main__":
    main()
