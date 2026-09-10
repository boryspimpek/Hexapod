from ik import calculate_leg_ik
from plot import visualize_leg
from config import l_coxa, l_femur, l_tibia


x, y, z = 0.0, 70.0, -50.0  

a, b, c = calculate_leg_ik(x, y, z, l_coxa, l_femur, l_tibia)
print(f"Calculated angles: Coxa={a:.1f}°, Femur={b:.1f}°, Tibia={c:.1f}°")

visualize_leg(x, y, z, l_coxa, l_femur, l_tibia)
