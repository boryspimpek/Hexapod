"""Gait timing, turning, switching, and controller integration without hardware."""
import math
import unittest
from unittest.mock import Mock, patch

import main
from robot import config as cfg
from robot.body import body_center
from robot.gait import gait_settings, next_gait
from robot.joystick import get_gait_button
from robot.motion import calculate_frame, initial_state, step_motion
from simulator import simulate, world_point


class GaitTests(unittest.TestCase):
    def test_crawl_lifts_one_leg_in_order_and_trot_lifts_diagonal_pairs(self):
        for gait, expected in (("crawl", [{"lf"}, {"lr"}, {"rf"}, {"rr"}]),
                               ("trot", [{"lf", "rr"}, {"lf", "rr"},
                                         {"lr", "rf"}, {"lr", "rf"}])):
            for phase, lifted in zip((.125, .375, .625, .875), expected):
                frame = calculate_frame(phase, 1, gait=gait)
                self.assertEqual({name for name, leg in frame["legs"].items()
                                  if leg["target"][2] > cfg.z_height + 1e-8}, lifted)

    def test_crawl_turns_follow_swing_and_stance_durations(self):
        center = body_center()
        settings = gait_settings("crawl")
        for name in cfg.LEGS:
            for local_phase, elapsed_phase, progress in ((0, .125, .5), (.25, .375, -.5)):
                phase = (local_phase - settings["phase_offsets"][name]) % 1
                frames = [calculate_frame(p % 1, 1, turn=1, gait="crawl")
                          for p in (phase, phase + elapsed_phase)]
                vectors = []
                for frame in frames:
                    point = world_point(name, frame["legs"][name]["target"], frame["body_offset"])
                    vectors.append((point[0] - center[0], point[1] - center[1]))
                a, b = vectors
                delta = math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1])
                self.assertAlmostEqual(delta, -math.radians(cfg.turn_step_angle) * progress)

    def test_moving_switch_returns_to_neutral_before_starting_new_gait(self):
        state = {**initial_state(), "ramp": 1, "phase": .17,
                 "direction": (1, 0), "turn": .5}
        original = dict(state)
        state, frame = step_motion(state, 1, 0, .02, gait="crawl", step_length=40)
        self.assertLess(state["ramp"], 1)
        self.assertEqual(state["gait"], "trot")
        self.assertEqual(original["ramp"], 1)
        for _ in range(30):
            state, frame = step_motion(state, 1, 0, .02, step_length=40)
            if state["gait"] == "crawl":
                break
        self.assertEqual(state["gait"], "crawl")
        self.assertEqual(state["ramp"], 0)
        self.assertEqual(state["phase"], 0)
        for name, leg in frame["legs"].items():
            self.assertEqual(leg["target"], cfg.p_start[name])
        state, _ = step_motion(state, 1, 0, .02, step_length=40)
        self.assertGreater(state["ramp"], 0)
        self.assertEqual(state["gait"], "crawl")

    def test_default_and_cycle_are_configurable(self):
        with patch.object(cfg, "DEFAULT_GAIT", "crawl"):
            self.assertEqual(initial_state()["gait"], "crawl")
            self.assertEqual(calculate_frame()["gait"], "crawl")
        self.assertEqual(next_gait("trot"), "crawl")
        self.assertEqual(next_gait("crawl"), "trot")

    def test_hardware_button_switches_once_per_press(self):
        frames = []
        with patch.object(main, "get_left_stick", side_effect=[(0, 0)] * 4 + [KeyboardInterrupt]), \
                patch.object(main, "get_right_stick", return_value=(0, 0)), \
                patch.object(main, "get_gait_button", side_effect=[True, True, False, True]), \
                patch.object(main, "get_r1", return_value=False), \
                patch.object(main, "get_dpad_vertical", return_value=0), \
                patch.object(main.time, "monotonic", side_effect=[i * .02 for i in range(6)]), \
                patch.object(main.time, "sleep"), patch("builtins.print"), \
                patch.object(main, "send_frame", side_effect=lambda sock, frame: frames.append(frame)):
            with self.assertRaises(KeyboardInterrupt):
                main.main_loop(Mock(), Mock())
        self.assertEqual([f["gait"] for f in frames], ["trot", "crawl", "crawl", "crawl", "trot"])

    def test_button_mapping_and_missing_button(self):
        controller = Mock()
        controller.get_numbuttons.return_value = 0
        self.assertFalse(get_gait_button(controller))
        controller.get_numbuttons.return_value = 20
        controller.get_button.return_value = 1
        self.assertTrue(get_gait_button(controller))
        controller.get_button.assert_called_once_with(cfg.GAIT_BUTTON)

    def test_simulator_button_is_edge_triggered(self):
        state = {}
        for pressed, expected in ((True, "crawl"), (True, "crawl"),
                                  (False, "crawl"), (True, "trot")):
            frame = simulate({**state, "gait_button": pressed})
            state = {k: frame[k] for k in ("gait", "requested_gait", "previous_gait_button")}
            self.assertEqual(frame["gait"], expected)

    def test_crawl_odometry_uses_three_support_feet(self):
        state = {"gait": "crawl", "requested_gait": "crawl"}
        for _ in range(200):
            frame = simulate({**state, "x": 1, "step_length": 40})
            contacts = frame["odometry"]["contacts"]
            self.assertEqual(len(contacts), 3)
            for name in contacts:
                self.assertAlmostEqual(frame["legs"][name]["target"][2], cfg.z_height)
            state = {k: frame[k] for k in ("gait", "requested_gait", "phase", "ramp",
                                           "direction", "turn", "odometry")}
        self.assertGreater(frame["odometry"]["position"][0] - body_center()[0], 50)

    def test_invalid_gaits_are_rejected(self):
        for value in ("missing", [], 3):
            with self.subTest(value=value), self.assertRaises(ValueError):
                simulate({"requested_gait": value})
        for ratio in (0, 1, float("nan")):
            with patch.dict(cfg.GAITS, {"bad": {"swing_ratio": ratio,
                                               "phase_offsets": cfg.LEG_PHASE_OFFSET}}):
                with self.assertRaises(ValueError):
                    gait_settings("bad")


if __name__ == "__main__":
    unittest.main()
