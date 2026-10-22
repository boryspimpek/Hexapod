from robot import config as cfg
from robot.kinematics import inverse_kinematics

x, y, z = 60, 130, -40  # Example target position in mm
ik = inverse_kinematics(x, y, z, cfg.l_coxa, cfg.l_femur, cfg.l_tibia)    
print(ik)  # Should print the joint angles for the given position
