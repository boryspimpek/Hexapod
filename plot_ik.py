import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import matplotlib.patches as mpatches
from ik import calculate_joints, inverse_kinematics
from config import l_coxa, l_femur, l_tibia

"""
Rysuje pojedynczą nogę quadrupeda w 3D i pozwala interaktywnie zmieniać pozycję stopy (x, y, z) za pomocą przycisków.
Używa funkcji calculate_joints() z ik.py do obliczenia pozycji stawów i kątów stawów na podstawie pozycji stopy. Można wybrać id serva, 
aby uwzględnić inwersję w zależności od montażu serwa.

"""

servo_id = 4 # inverted == True, dla serwa 4 (coxa prawej przedniej nogi) w config.py` 

def draw_leg(ax, fig, x, y, z, l_coxa, l_femur, l_tibia):
    points, angles, error_msg = calculate_joints(servo_id, x, y, z, l_coxa, l_femur, l_tibia)
    p0, p_hip, p_knee, p_foot = points
    c, f, t = angles

    ax.clear()

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    ax.plot(xs, ys, zs, "o-", linewidth=4, markersize=8, color="#1f77b4")
    ax.scatter(*p0, color="black", s=100)
    ax.scatter(*p_hip, color="orange", s=80)
    ax.scatter(*p_knee, color="green", s=80)
    ax.scatter(*p_foot, color="red", s=100)

    ax.set_title("Kinematyka odwrotna nogi Quadrupeda", fontsize=13, fontweight="bold", pad=15)
    ax.set_xlabel("Oś X [mm]")
    ax.set_ylabel("Oś Y [mm]")
    ax.set_zlabel("Oś Z [mm]")

    max_range = max(l_coxa + l_femur + l_tibia, abs(x), abs(y), abs(z)) * 1.1
    ax.set_xlim([max_range / 2, -max_range / 2])  # odwrócenie kierunku osi X
    ax.set_ylim([-max_range / 2, max_range / 2])
    ax.set_zlim([-max_range / 2, max_range / 2])

    # --- Wyczyść stare elementy panelu (tekst + tło) ---
    for txt in fig.texts:
        txt.remove()
    panel_bg = getattr(fig, "_info_panel", None)
    if panel_bg is not None:
        fig.patches.remove(panel_bg)

    # --- Tło panelu informacyjnego ---
    panel_x, panel_w = 0.75, 0.22
    panel_bg = mpatches.FancyBboxPatch(
        (panel_x, 0.45), panel_w, 0.40,
        boxstyle="round,pad=0.015",
        transform=fig.transFigure,
        facecolor="#f0f0f0", edgecolor="#999999", linewidth=1,
        zorder=0,
    )
    fig.patches.append(panel_bg)
    fig._info_panel = panel_bg

    text_x = panel_x + 0.02
    y_cursor = 0.80
    line_h = 0.035

    def add_line(text, color="black", bold=False, size=10):
        nonlocal y_cursor
        fig.text(text_x, y_cursor, text, fontsize=size, family="monospace",
                  color=color, fontweight="bold" if bold else "normal",
                  verticalalignment="top")
        y_cursor -= line_h

    def add_legend_row(dot_color, label, value):
        nonlocal y_cursor
        fig.text(text_x, y_cursor, "●", fontsize=12, color=dot_color,
                  verticalalignment="top")
        fig.text(text_x + 0.025, y_cursor, f"{label:<7}= {value:7.1f}°",
                  fontsize=10, family="monospace", verticalalignment="top")
        y_cursor -= line_h

    add_line("POZYCJA STOPY [mm]", bold=True)
    add_line(f"  X = {x:7.1f}")
    add_line(f"  Y = {y:7.1f}")
    add_line(f"  Z = {z:7.1f}")
    y_cursor -= line_h * 0.5

    add_line("KĄTY STAWÓW [°]", bold=True)
    add_legend_row("black", "Coxa", c)
    add_legend_row("orange", "Femur", f)
    add_legend_row("green", "Tibia", t)


    if error_msg:
        y_cursor -= line_h * 0.5
        fig.text(text_x, y_cursor, f"⚠ BŁĄD:\n{error_msg}",
                  fontsize=10, color="darkred", fontweight="bold",
                  wrap=True, verticalalignment="top")

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
    """Otwiera interaktywne okno 3D z nogą quadrupeda i przyciskami sterującymi."""
    position = {"x": x, "y": y, "z": z}

    fig = plt.figure(figsize=(13, 9))
    plt.subplots_adjust(bottom=0.22, right=0.72)
    ax = fig.add_subplot(111, projection="3d")

    def redraw():
        draw_leg(ax, fig, position["x"], position["y"], position["z"],
                  l_coxa, l_femur, l_tibia)

    redraw()
    _buttons = setup_buttons(position, step, redraw)  

    plt.show()


def main():
    x, y, z = 0.0, 160.0, -50.0  

    a, b, c = inverse_kinematics(x, y, z, l_coxa, l_femur, l_tibia)
    print(f"Calculated angles: Coxa={a:.1f}°, Femur={b:.1f}°, Tibia={c:.1f}°")

    visualize_leg(x, y, z, l_coxa, l_femur, l_tibia)

if __name__ == "__main__":
    main()
