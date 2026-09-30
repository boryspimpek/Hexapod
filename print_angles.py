# dump_cycle.py
import csv
from config import (LEGS, SERVO_ID, LEG_PHASE_OFFSET,
                    l_coxa, l_femur, l_tibia, step_length, step_height, p_start)
from ik import inverse_kinematics
from gait import JOINTS, calculate_trajectory, correct_angle, apply_offsets

N_STEPS = 20          # ile próbek na cały cykl
JX, JY = 1.0, 0.0     # symulowane wychylenie drążka (1,0 = do przodu)

rows = []            # do CSV: jedna linia = (faza, noga, x,y,z, kąty surowe i wysyłane)
by_servo = {}        # {faza_idx: {servo_id: (kąt, przycięty)}}

for i in range(N_STEPS):
    phase = i / N_STEPS
    by_servo[i] = {}
    for leg in LEGS:
        lp = (phase + LEG_PHASE_OFFSET[leg]) % 1.0
        x, y, z = calculate_trajectory(lp, step_length, step_height, p_start[leg], JX, JY)
        raw = inverse_kinematics(x, y, z, l_coxa, l_femur, l_tibia)
        out = []
        for joint, ang in zip(("coxa", "femur", "tibia"), raw):
            sid = LEGS[leg][joint][SERVO_ID]
            final = correct_angle(sid, ang)
            clamped = final != apply_offsets(sid, ang)
            by_servo[i][sid] = (final, clamped)
            out.append((sid, ang, final))
        rows.append([round(phase, 3), leg, round(x, 1), round(y, 1), round(z, 1)] +
                    [v for _, r, f in out for v in (round(r, 1), round(f, 1))])

# ---- Tabela 1: wszystkie serwa naraz (wiersz = faza) ----
ids = sorted(JOINTS)
print("Kąty wysyłane do serw ('*' = przycięte przez LIMITS)\n")
print("faza  " + "".join(f"S{sid:<6}" for sid in ids))
for i in range(N_STEPS):
    line = f"{i / N_STEPS:4.2f}  "
    for sid in ids:
        a, c = by_servo[i][sid]
        line += f"{a:6.1f}{'*' if c else ' '} "
    print(line)

# ---- Tabela 2: min / max / zakres per serwo ----
print("\nSerwo   min     max     zakres  przycięć")
for sid in ids:
    vals = [by_servo[i][sid][0] for i in range(N_STEPS)]
    clip = sum(by_servo[i][sid][1] for i in range(N_STEPS))
    print(f"S{sid:<5} {min(vals):6.1f}  {max(vals):6.1f}  {max(vals)-min(vals):6.1f}   {clip}")

# ---- CSV ----
with open("cycle.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["phase", "leg", "x", "y", "z",
                "coxa_raw", "coxa_out", "femur_raw", "femur_out", "tibia_raw", "tibia_out"])
    w.writerows(rows)
print("\nZapisano cycle.csv")