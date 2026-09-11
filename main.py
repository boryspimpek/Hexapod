from ik import calculate_joints, calculate_leg_ik
from plot import visualize_leg
from config import l_coxa, l_femur, l_tibia


x, y, z = 0.0, 110.0, -70.0  

a, b, c = calculate_leg_ik(x, y, z, l_coxa, l_femur, l_tibia)
print(f"Calculated angles: Coxa={a:.1f}°, Femur={b:.1f}°, Tibia={c:.1f}°")

calculate_joints(x, y, z, l_coxa, l_femur, l_tibia)  # This will print the joint positions and angles




# visualize_leg(x, y, z, l_coxa, l_femur, l_tibia)
