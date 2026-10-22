"""Body translation, controller wiring, and fixed-foot simulator checks."""
import unittest
from unittest.mock import Mock, patch

import main
from robot import config as cfg
from robot.joystick import get_right_stick
from simulator import simulate


class BodyShiftTests(unittest.TestCase):
    def test_body_moves_while_world_foot_trajectory_stays_fixed(self):
        for walking in ({}, {"direction": (.6, .8), "ramp": 1, "phase": .125}):
            baseline = simulate(walking)
            for right_x, right_y in ((-1, 0), (1, 0), (0, 1), (0, -1), (.4, -.7)):
                with self.subTest(walking=walking, right_x=right_x, right_y=right_y):
                    frame = simulate({**walking, "right_x": right_x, "right_y": right_y, "r1": True})
                    dx, dy = right_y * cfg.BODY_SHIFT_MAX, -right_x * cfg.BODY_SHIFT_MAX
                    for name, leg in frame["legs"].items():
                        before = baseline["legs"][name]
                        side = 1 if name.startswith("l") else -1
                        self.assertAlmostEqual(leg["target"][0], before["target"][0] - dx)
                        self.assertAlmostEqual(leg["target"][1], before["target"][1] - side * dy)
                        for actual, expected in zip(leg["target_world"], before["target_world"]):
                            self.assertAlmostEqual(actual, expected)
                        origin = cfg.LEG_ORIGINS[name]
                        self.assertEqual(frame["origins"][name],
                                         [origin[0] + dx, origin[1] + dy, origin[2]])
                        self.assertEqual(leg["points"][0], frame["origins"][name])
                    self.assertEqual(frame["phase"], baseline["phase"])
                    self.assertEqual(frame["ramp"], baseline["ramp"])
            self.assertEqual(simulate(walking), baseline)

    def test_right_stick_axes_and_deadzone(self):
        controller = Mock()
        controller.get_numaxes.return_value = 4
        controller.get_axis.side_effect = {cfg.RIGHT_STICK_X_AXIS: .575,
                                           cfg.RIGHT_STICK_Y_AXIS: -.575}.__getitem__
        for value in get_right_stick(controller):
            self.assertAlmostEqual(value, .5)
        controller.get_axis.side_effect = lambda axis: .05
        self.assertEqual(get_right_stick(controller), (0, 0))
        controller.get_numaxes.return_value = 2
        controller.get_axis.reset_mock()
        self.assertEqual(get_right_stick(controller), (0, 0))
        controller.get_axis.assert_not_called()

    def run_controller(self, sticks):
        frames = []
        with patch.object(main, "get_left_stick", return_value=(0, 0)), \
                patch.object(main, "get_right_stick", side_effect=[*sticks, KeyboardInterrupt]), \
                patch.object(main, "get_r1", return_value=True), \
                patch.object(main, "get_dpad_vertical", return_value=0), \
                patch.object(main, "send_frame", side_effect=lambda sock, frame: frames.append(frame)), \
                patch.object(main.time, "sleep"), patch("builtins.print"):
            with self.assertRaises(KeyboardInterrupt):
                main.main_loop(Mock(), Mock())
        return frames

    def test_controller_translation_and_release_without_walking(self):
        frames = self.run_controller([(-1, 0), (0, 1), (0, 0)])
        self.assertEqual([frame["body_offset"] for frame in frames],
                         [(0, 0, 0), (0, cfg.BODY_SHIFT_MAX, 0),
                          (cfg.BODY_SHIFT_MAX, 0, 0), (0, 0, 0)])
        self.assertEqual(frames[0], frames[-1])

    def test_unreachable_body_request_keeps_previous_position(self):
        with patch.object(cfg, "BODY_SHIFT_MAX", 500):
            frames = self.run_controller([(1, 0), (0, 0)])
        self.assertEqual(frames[0], frames[1])
        self.assertEqual(frames[0], frames[2])

    def test_simulator_rejects_invalid_stick_input(self):
        for key in ("right_x", "right_y"):
            for value in (-1.01, 1.01, float("nan"), float("inf")):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    simulate({key: value})


if __name__ == "__main__":
    unittest.main()
