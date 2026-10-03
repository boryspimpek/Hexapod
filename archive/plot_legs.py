# plot_legs.py
import math
import matplotlib.pyplot as plt
from config import (LEGS, SERVO_ID, LEG_PHASE_OFFSET, INVERTED, LIMITS, TRIM,
                    l_coxa, l_femur, l_tibia, step_length, step_height, p_start)
from ik import inverse_kinematics

N_STEPS = 200         # gęstość próbkowania cyklu (więcej = gładsze wykresy)
JX, JY = 1.0, 0.0     # symulowane wychylenie drążka (1,0 = do przodu)

COLORS = {"coxa": "tab:blue", "femur": "tab:orange", "tibia": "tab:green"}


def calc_dir(jx, jy):
    m = math.hypot(jx, jy)
    return (0.0, 0.0) if m < 1e-6 else (jx / m, jy / m)


def calculate_trajectory(phase, step_length, step_height, p_start, jx, jy):
    swing = 0.5
    d = calc_dir(jx, jy)
    p_end = (p_start[0] + d[0] * step_length, p_start[1] + d[1] * step_length, p_start[2])
    if phase < swing:
        t = phase / swing
        return (p_start[0] + (p_end[0] - p_start[0]) * t,
                p_start[1] + (p_end[1] - p_start[1]) * t,
                p_start[2] + step_height * math.sin(t * math.pi))
    t = (phase - swing) / (1.0 - swing)
    return (p_end[0] + (p_start[0] - p_end[0]) * t,
            p_end[1] + (p_start[1] - p_end[1]) * t,
            p_start[2])


def correct_angle(data, angle_deg):
    lo, hi = data[LIMITS]
    a = (180 - angle_deg) if data[INVERTED] else angle_deg
    a += data[TRIM]
    clamped = max(lo, min(hi, a))
    return clamped, clamped != a


# ---- obliczenia ----
phases = [i / N_STEPS for i in range(N_STEPS)]
data = {}   # data[leg] = {"pos": [(x,y,z)...], "coxa": [(kąt, przycięty)...], ...}

for leg in LEGS:
    d = {"pos": [], "coxa": [], "femur": [], "tibia": []}
    for ph in phases:
        lp = (ph + LEG_PHASE_OFFSET[leg]) % 1.0
        x, y, z = calculate_trajectory(lp, step_length, step_height, p_start, JX, JY)
        d["pos"].append((x, y, z))
        raw = inverse_kinematics(x, y, z, l_coxa, l_femur, l_tibia)
        for joint, ang in zip(("coxa", "femur", "tibia"), raw):
            d[joint].append(correct_angle(LEGS[leg][joint], ang))
    data[leg] = d

# ---- wykresy: jeden wiersz na nogę, 3 kolumny ----
n = len(LEGS)
fig, axes = plt.subplots(n, 3, figsize=(16, 3.6 * n), squeeze=False)

for row, leg in enumerate(LEGS):
    d = data[leg]
    xs = [p[0] for p in d["pos"]]
    ys = [p[1] for p in d["pos"]]
    zs = [p[2] for p in d["pos"]]

    # 1) pozycja stopy w czasie
    ax = axes[row][0]
    ax.plot(phases, xs, label="x")
    ax.plot(phases, ys, label="y")
    ax.plot(phases, zs, label="z")
    ax.axvline(0.5, color="gray", ls=":", lw=0.8)   # granica swing / stance
    ax.set_title(f"{leg}: pozycja stopy")
    ax.set_ylabel("mm")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    # 2) kąty serw w czasie (po korekcie), limity i przycięcia
    ax = axes[row][1]
    for joint in ("coxa", "femur", "tibia"):
        angles = [a for a, _ in d[joint]]
        sid = LEGS[leg][joint][SERVO_ID]
        ax.plot(phases, angles, color=COLORS[joint], label=f"{joint} (S{sid})")
        lo, hi = LEGS[leg][joint][LIMITS]
        ax.axhline(lo, color=COLORS[joint], ls="--", lw=0.7, alpha=0.6)
        ax.axhline(hi, color=COLORS[joint], ls="--", lw=0.7, alpha=0.6)
        cx = [p for p, (_, c) in zip(phases, d[joint]) if c]
        cy = [a for a, c in d[joint] if c]
        if cx:
            ax.scatter(cx, cy, color="red", marker="x", s=25, zorder=5)
    ax.axvline(0.5, color="gray", ls=":", lw=0.8)
    ax.set_title(f"{leg}: kąty serw (przerywane = LIMITS, x = przycięte)")
    ax.set_ylabel("stopnie")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)

    # 3) tor stopy z boku (x-z)
    ax = axes[row][2]
    sc = ax.scatter(xs, zs, c=phases, cmap="viridis", s=10)
    ax.set_title(f"{leg}: tor stopy (x-z), kolor = faza")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("z [mm]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    fig.colorbar(sc, ax=ax, label="faza")

fig.tight_layout()
plt.show()