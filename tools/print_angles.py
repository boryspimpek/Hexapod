from robot import config as cfg
from robot.motion import calculate_frame, JOINT_NAMES
from robot.servos import JOINTS, apply_offsets

N_STEPS = 20
DIRECTION = (1.0, 0.0)


def sample_cycle(n_steps=N_STEPS, direction=DIRECTION):
    by_servo = {}
    for i in range(n_steps):
        frame = calculate_frame(i / n_steps, 1.0, direction)
        by_servo[i] = {}
        for name, leg in frame["legs"].items():
            for joint, raw in zip(JOINT_NAMES, leg["raw_angles"]):
                sid = cfg.LEGS[name][joint][cfg.SERVO_ID]
                final = frame["servos"][sid]
                by_servo[i][sid] = (final, final != apply_offsets(sid, raw))
    return by_servo


def main():
    by_servo = sample_cycle()
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


if __name__ == "__main__":
    main()
