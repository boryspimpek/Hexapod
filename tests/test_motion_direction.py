"""Check foot motion in body coordinates across mirrored leg frames."""
import math
import unittest

from robot import config as cfg
from robot.motion import calculate_frame
from simulator import world_point


class MotionDirectionTests(unittest.TestCase):
    def test_swing_and_stance_follow_body_direction(self):
        # Compare each leg at the same local phase despite gait phase offsets.
        for direction in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1)):
            magnitude = math.hypot(*direction)
            for name in cfg.LEGS:
                for local_phase, travel_sign in ((.125, 1), (.625, -1)):
                    with self.subTest(direction=direction, leg=name, phase=local_phase):
                        phase = (local_phase - cfg.LEG_PHASE_OFFSET[name]) % 1
                        before = calculate_frame(phase, 1, direction, step_length=40)
                        after = calculate_frame(phase + .125, 1, direction, step_length=40)
                        a = world_point(name, before["legs"][name]["target"])
                        b = world_point(name, after["legs"][name]["target"])
                        # Body +X is forward, body +Y is left; input +Y is right.
                        self.assertAlmostEqual(b[0] - a[0],
                                               travel_sign * 10 * direction[0] / magnitude)
                        self.assertAlmostEqual(b[1] - a[1],
                                               -travel_sign * 10 * direction[1] / magnitude)


if __name__ == "__main__":
    unittest.main()
