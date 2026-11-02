"""Crawl counter-shift directions, continuity, and fixed ground targets."""
import unittest
from unittest.mock import patch

from robot import config as cfg
from robot.gait import gait_settings
from robot.motion import calculate_frame, initial_state, step_motion
from simulator import simulate


class CrawlBodyShiftTests(unittest.TestCase):
    def assert_vector_equal(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=7)

    def test_body_moves_opposite_each_lifted_leg(self):
        # World axes: X forward, Y left.
        for phase, leg, expected in ((.125, "lf", (-10, -10, 0)),
                                     (.375, "lr", (10, -10, 0)),
                                     (.625, "rf", (-10, 10, 0)),
                                     (.875, "rr", (10, 10, 0))):
            frame = calculate_frame(phase, 1, gait="crawl")
            self.assert_vector_equal(frame["body_offset"], expected)
            self.assertGreater(frame["legs"][leg]["target"][2], cfg.z_height)

    def test_continuous_at_leg_exchanges_and_cycle_wrap(self):
        for phase in (0, .125, .25, .375, .5, .625, .75, .875):
            before = calculate_frame((phase - 1e-6) % 1, 1, gait="crawl")
            after = calculate_frame((phase + 1e-6) % 1, 1, gait="crawl")
            for a, b in zip(before["body_offset"], after["body_offset"]):
                self.assertLess(abs(a - b), .001)

    def test_ramp_scales_shift_and_trot_stays_centered(self):
        full = calculate_frame(.125, 1, gait="crawl")["body_offset"]
        half = calculate_frame(.125, .5, gait="crawl")["body_offset"]
        self.assert_vector_equal(half, tuple(v * .5 for v in full))
        self.assertEqual(calculate_frame(.125, 0, gait="crawl")["body_offset"], (0, 0, 0))
        self.assertEqual(calculate_frame(.125, 1, gait="trot")["body_offset"], (0, 0, 0))

    def test_manual_pose_adds_to_shift_without_moving_world_targets(self):
        for controls in ({"x": 1, "right_x": .4},
                         {"r1": True, "x": .3, "y": -.4, "right_x": .2, "right_y": .3}):
            request = {"gait": "crawl", "phase": .125, "ramp": 1,
                       "direction": (1, 0), **controls}
            enabled = simulate(request)
            settings = {**cfg.GAITS["crawl"], "body_shift": (0, 0)}
            with patch.dict(cfg.GAITS, {"crawl": settings}):
                disabled = simulate(request)
            for name in cfg.LEGS:
                self.assert_vector_equal(enabled["legs"][name]["target_world"],
                                         disabled["legs"][name]["target_world"])
            for i in range(3):
                self.assertAlmostEqual(enabled["body_offset"][i],
                                       disabled["body_offset"][i] + enabled["gait_body_offset"][i])

    def test_stopping_returns_to_center(self):
        state = {**initial_state(), "gait": "crawl", "requested_gait": "crawl",
                 "phase": .125, "ramp": 1, "direction": (1, 0)}
        for _ in range(30):
            state, frame = step_motion(state, 0, 0, .02)
        self.assertEqual(state["ramp"], 0)
        self.assertEqual(frame["body_offset"], (0, 0, 0))

    def test_sway_does_not_create_false_odometry(self):
        state = {"gait": "crawl"}
        for _ in range(200):
            frame = simulate({**state, "x": 1, "step_length": 0})
            self.assert_vector_equal(frame["odometry"]["position"], frame["reference_center"][:2])
            self.assertAlmostEqual(frame["odometry"]["yaw"], 0)
            state = {k: frame[k] for k in ("gait", "phase", "ramp", "direction", "turn", "odometry")}

    def test_shift_settings_validate_and_zero_disables(self):
        crawl = cfg.GAITS["crawl"]
        with patch.dict(cfg.GAITS, {"crawl": {**crawl, "body_shift": (0, 0)}}):
            self.assertEqual(calculate_frame(.125, 1, gait="crawl")["body_offset"], (0, 0, 0))
        for invalid in ((-1, 2), (float("nan"), 2), (1,), "bad"):
            with patch.dict(cfg.GAITS, {"crawl": {**crawl, "body_shift": invalid}}):
                with self.assertRaises(ValueError):
                    gait_settings("crawl")
        with patch.dict(cfg.GAITS, {"crawl": {**crawl, "swing_ratio": .5}}):
            with self.assertRaises(ValueError):
                gait_settings("crawl")


if __name__ == "__main__":
    unittest.main()
