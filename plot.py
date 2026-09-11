import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from ik import calculate_joints

def draw_leg(ax, fig, x, y, z, l_coxa, l_femur, l_tibia):
    """Przelicza IK dla podanej pozycji i (od)rysowuje wykres 3D nogi."""
    points, angles, error_msg = calculate_joints(x, y, z, l_coxa, l_femur, l_tibia)
    p0, p_hip, p_knee, p_foot = points
    c, f, t = angles

    ax.clear()

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    ax.plot(xs, ys, zs, "o-", linewidth=4, markersize=8,
            color="#1f77b4", label="Segmenty nogi")
    ax.scatter(*p0, color="black", s=100, label="Coxa")
    ax.scatter(*p_hip, color="orange", s=80, label="Femur")
    ax.scatter(*p_knee, color="green", s=80, label="Tibia")
    ax.scatter(*p_foot, color="red", s=100, label="Stopa")

    ax.set_title("Kinematyka odwrotna nogi Hexapoda", fontsize=13, fontweight="bold", pad=15)

    ax.set_xlabel("Oś X [mm]")
    ax.set_ylabel("Oś Y [mm]")
    ax.set_zlabel("Oś Z [mm]")

    max_range = max(l_coxa + l_femur + l_tibia, abs(x), abs(y), abs(z)) * 1.1
    ax.set_xlim([-max_range / 2, max_range / 2])
    ax.set_ylim([-max_range / 2, max_range / 2])
    ax.set_zlim([-max_range / 2, max_range / 2])

    # Legenda pozioma, pod wykresem
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08),
              ncol=5, fontsize=9, frameon=True, framealpha=0.9)

    # --- Panel informacyjny ---
    info_lines = [
        "POZYCJA STOPY [mm]",
        f"  X = {x:7.1f}",
        f"  Y = {y:7.1f}",
        f"  Z = {z:7.1f}",
        "",
        "KĄTY STAWÓW [°]",
        f"  Coxa  = {c:7.1f}",
        f"  Femur = {f:7.1f}",
        f"  Tibia = {t:7.1f}",
    ]
    info_text = "\n".join(info_lines)

    for txt in fig.texts:
        txt.remove()

    fig.text(
        0.78, 0.55, info_text,
        fontsize=11, family="monospace",
        verticalalignment="center",
        bbox=dict(boxstyle="round,pad=0.6", facecolor="#f0f0f0", edgecolor="#999999"),
    )

    if error_msg:
        fig.text(
            0.78, 0.30, f"⚠ BŁĄD:\n{error_msg}",
            fontsize=10, color="darkred", fontweight="bold",
            wrap=True, verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#ffe0e0", edgecolor="darkred"),
        )

    fig.canvas.draw_idle()

def make_move_callback(position, axis, delta, redraw):
    """Zwraca callback przycisku przesuwający współrzędną `axis` o `delta`."""
    def callback(event):
        position[axis] += delta
        redraw()
    return callback


def setup_buttons(position, step, redraw):
    """Tworzy przyciski X/Y/Z +/- i podpina pod nie callbacki. Zwraca listę
    obiektów Button (trzeba je przechować, inaczej przestają działać)."""
    specs = [
        ("X +", 0.15, 0.12, "x", +step),
        ("X -", 0.15, 0.04, "x", -step),
        ("Y +", 0.45, 0.12, "y", +step),
        ("Y -", 0.45, 0.04, "y", -step),
        ("Z +", 0.75, 0.12, "z", +step),
        ("Z -", 0.75, 0.04, "z", -step),
    ]

    buttons = []
    for label, x_pos, y_pos, axis, delta in specs:
        ax_btn = plt.axes([x_pos, y_pos, 0.08, 0.06])
        btn = Button(ax_btn, label)
        btn.on_clicked(make_move_callback(position, axis, delta, redraw))
        buttons.append(btn)
    return buttons


def visualize_leg(x, y, z, l_coxa, l_femur, l_tibia, step=5.0):
    """Otwiera interaktywne okno 3D z nogą hexapoda i przyciskami sterującymi."""
    position = {"x": x, "y": y, "z": z}

    fig = plt.figure(figsize=(13, 9))
    plt.subplots_adjust(bottom=0.30, right=0.72)  # więcej miejsca: przyciski + legenda pod spodem
    ax = fig.add_subplot(111, projection="3d")

    def redraw():
        draw_leg(ax, fig, position["x"], position["y"], position["z"],
                  l_coxa, l_femur, l_tibia)

    redraw()
    _buttons = setup_buttons(position, step, redraw)  # noqa: F841 (referencje muszą żyć)

    plt.show()