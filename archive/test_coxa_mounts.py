"""Mount orientation, IK compensation, and simulated servo limits."""
import math
import unittest

from robot import config as cfg
from robot.kinematics import forward_kinematics, inverse_kinematics
from robot.servos import apply_offsets, remove_offsets
from simulator import joint_points, simulate, world_point


class CoxaMountTests(unittest.TestCase):
    def assert_point_equal(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=8)

    def test_neutral_coxa_points_front_forward_and_rear_backward(self):
        for name in cfg.LEGS:
            origin, tip = joint_points(name, (cfg.COXA_ZERO, 120, 60))[:2]
            forward = 1 if name.endswith("f") else -1
            outward = 1 if name.startswith("l") else -1
            reach = cfg.l_coxa / math.sqrt(2)
            self.assert_point_equal(tip, (origin[0] + forward * reach,
                                          origin[1] + outward * reach, origin[2]))

    def test_ik_compensates_mount_and_fk_recovers_targets(self):
        for name, mount in cfg.COXA_MOUNT_ANGLES.items():
            for target in ((0, 160, -40), (45, 140, -80), (-35, 150, -60)):
                with self.subTest(name=name, target=target):
                    args = (*target, cfg.l_coxa, cfg.l_femur, cfg.l_tibia)
                    angles = inverse_kinematics(*args, mount_angle=mount)
                    self.assertAlmostEqual(angles[0], inverse_kinematics(*args)[0] - mount)
                    foot = forward_kinematics(angles, cfg.l_coxa, cfg.l_femur,
                                              cfg.l_tibia, mount_angle=mount)[-1]
                    self.assert_point_equal(foot, target)

    def test_simulator_uses_clipped_servo_angle_with_rotated_mount(self):
        frame = simulate({"x_offset_front": -150, "y_offset_front": 50,
                          "x_offset_rear": 150, "y_offset_rear": 50})
        for name, leg in frame["legs"].items():
            with self.subTest(name=name):
                spec = cfg.LEGS[name]["coxa"]
                sid = spec[cfg.SERVO_ID]
                requested = apply_offsets(sid, leg["raw_angles"][0])
                lo, hi = spec[cfg.LIMITS]
                command = leg["servo_angles"]["coxa"]
                self.assertEqual(command, max(lo, min(hi, requested)))
                self.assertTrue(leg["limited"][0])
                coxa = math.radians(remove_offsets(sid, command) - cfg.COXA_ZERO
                                    + cfg.COXA_MOUNT_ANGLES[name])
                tip = (cfg.l_coxa * math.sin(coxa), cfg.l_coxa * math.cos(coxa), 0)
                self.assert_point_equal(leg["points"][1], world_point(name, tip))


if __name__ == "__main__":
    unittest.main()
