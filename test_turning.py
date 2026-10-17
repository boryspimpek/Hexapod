"""Yaw geometry, gait lifecycle, and R1 mode switching without hardware."""
import math
import unittest
from unittest.mock import Mock, patch

import main
from robot import config as cfg
from robot.joystick import get_r1
from robot.motion import calculate_frame, initial_state, step_motion
from simulator import simulate, world_point


class TurningTests(unittest.TestCase):
    def test_pure_turn_follows_circular_arcs_with_correct_sign(self):
        center = tuple((min(p[i] for p in cfg.LEG_ORIGINS.values())
                        + max(p[i] for p in cfg.LEG_ORIGINS.values())) / 2 for i in (0, 1))
        for turn in (-1, .5, 1):
            for name in cfg.LEGS:
                for local_phase, sign in ((0, -1), (.5, 1)):
                    with self.subTest(turn=turn, leg=name, phase=local_phase):
                        phase = (local_phase - cfg.LEG_PHASE_OFFSET[name]) % 1
                        a = calculate_frame(phase, 1, turn=turn)
                        b = calculate_frame((phase + .125) % 1, 1, turn=turn)
                        p, q = (world_point(name, frame["legs"][name]["target"])
                                for frame in (a, b))
                        px, py = p[0] - center[0], p[1] - center[1]
                        qx, qy = q[0] - center[0], q[1] - center[1]
                        self.assertAlmostEqual(math.hypot(px, py), math.hypot(qx, qy))
                        angle = math.atan2(px * qy - py * qx, px * qx + py * qy)
                        self.assertAlmostEqual(angle, sign * math.radians(cfg.turn_step_angle) * turn / 4)

    def test_turn_starts_and_stops_without_left_stick(self):
        state = initial_state()
        before = dict(state)
        state, frame = step_motion(state, 0, 0, .02, turn=1)
        self.assertEqual(before, initial_state())
        self.assertGreater(state["ramp"], 0)
        self.assertGreater(state["phase"], 0)
        self.assertEqual(state["direction"], (0, 0))
        self.assertEqual(state["turn"], 1)
        state, _ = step_motion(state, 0, 0, .1, turn=1)
        ramp = state["ramp"]
        state, _ = step_motion(state, 0, 0, .02)
        self.assertLess(state["ramp"], ramp)
        self.assertEqual(state["turn"], 1)  # preserve yaw while ramping down
        state, frame = step_motion(state, 0, 0, 1)
        self.assertEqual(state["ramp"], 0)
        self.assertEqual(state["phase"], 0)
        for name, leg in frame["legs"].items():
            for actual, expected in zip(leg["target"], cfg.p_start[name]):
                self.assertAlmostEqual(actual, expected)

    def test_simulator_modes_and_vertical_axis(self):
        self.assertEqual(simulate({"right_y": 1}), simulate({}))
        turn = simulate({"right_x": -1})
        self.assertEqual(turn["body_offset"], (0, 0, 0))
        self.assertEqual(turn["turn"], -1)
        self.assertGreater(turn["ramp"], 0)
        shift = simulate({"right_x": -1, "r1": True, "turn": 1})
        self.assertEqual(shift["turn"], 0)
        self.assertEqual(shift["ramp"], 0)
        self.assertEqual(shift["body_offset"], (0, cfg.BODY_SHIFT_MAX, 0))
        combined = simulate({"x": 1, "right_x": 1})
        self.assertEqual(combined["direction"], (1, 0))
        self.assertEqual(combined["turn"], 1)
        self.assertNotEqual(combined["legs"]["lf"]["target"], turn["legs"]["lf"]["target"])

    def test_hardware_mode_switch_matches_simulator(self):
        frames = []
        with patch.object(main, "get_left_stick", return_value=(0, 0)), \
                patch.object(main, "get_right_stick", side_effect=[(-1, .8), (.5, .5), (0, 1), KeyboardInterrupt]), \
                patch.object(main, "get_r1", side_effect=[False, True, False]), \
                patch.object(main, "get_dpad_vertical", return_value=0), \
                patch.object(main.time, "monotonic", side_effect=[0, .02, .04, .06, .08]), \
                patch.object(main.time, "sleep"), patch("builtins.print"), \
                patch.object(main, "send_frame", side_effect=lambda sock, frame: frames.append(frame)):
            with self.assertRaises(KeyboardInterrupt):
                main.main_loop(Mock(), Mock())
        a = simulate({"right_x": -1, "right_y": .8})
        b = simulate({**{k: a[k] for k in ("phase", "ramp", "direction", "turn")},
                      "right_x": .5, "right_y": .5, "r1": True})
        c = simulate({**{k: b[k] for k in ("phase", "ramp", "direction", "turn")}, "right_y": 1})
        for hardware, preview in zip(frames[1:], (a, b, c)):
            self.assertEqual(hardware["body_offset"], preview["body_offset"])
            for name in cfg.LEGS:
                for actual, expected in zip(hardware["legs"][name]["target"], preview["legs"][name]["target"]):
                    self.assertAlmostEqual(actual, expected)

    def test_r1_button_and_missing_button(self):
        controller = Mock()
        controller.get_numbuttons.return_value = cfg.R1_BUTTON + 1
        controller.get_button.return_value = 1
        self.assertTrue(get_r1(controller))
        controller.get_button.assert_called_once_with(cfg.R1_BUTTON)
        controller.get_button.return_value = 0
        self.assertFalse(get_r1(controller))
        controller.get_numbuttons.return_value = 0
        controller.get_button.reset_mock()
        self.assertFalse(get_r1(controller))
        controller.get_button.assert_not_called()

    def test_simulator_rejects_invalid_mode_and_turn_settings(self):
        for request in ({"r1": "false"}, {"turn": float("nan")}, {"turn": 2},
                        {"turn_step_angle": -1}, {"turn_step_angle": 31}):
            with self.subTest(request=request), self.assertRaises(ValueError):
                simulate(request)


if __name__ == "__main__":
    unittest.main()
