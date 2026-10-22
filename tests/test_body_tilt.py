"""Body pose compensation and controller/simulator parity without hardware."""
import unittest
from unittest.mock import Mock, patch

import main
from robot import config as cfg
from robot.kinematics import forward_kinematics
from robot.motion import calculate_frame
from simulator import simulate, world_point


class BodyTiltTests(unittest.TestCase):
    def assert_point_equal(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=8)

    def test_inverse_pose_keeps_world_feet_fixed_including_combined_axes(self):
        for roll, pitch in ((8, 0), (-8, 0), (0, 8), (0, -8), (8, -8), (-8, 8)):
            for body_x, body_y in ((0, 0), (12, -16)):
                for gait in ({}, {"phase": .125, "ramp": .7, "direction": (.6, .8), "turn": .3}):
                    with self.subTest(roll=roll, pitch=pitch, shift=(body_x, body_y), gait=gait):
                        baseline = calculate_frame(**gait)
                        tilted = calculate_frame(**gait, body_roll=roll, body_pitch=pitch,
                                                 body_x=body_x, body_y=body_y)
                        for name, leg in tilted["legs"].items():
                            expected = world_point(name, baseline["legs"][name]["target"])
                            self.assert_point_equal(world_point(name, leg["target"],
                                                    tilted["body_offset"], tilted["body_rotation"]),
                                                    expected)
                            foot = forward_kinematics(leg["raw_angles"], cfg.l_coxa,
                                                      cfg.l_femur, cfg.l_tibia)[-1]
                            self.assert_point_equal(world_point(name, foot, tilted["body_offset"],
                                                                tilted["body_rotation"]), expected)

    def test_simulator_tilt_signs_and_stationary_targets(self):
        baseline = simulate({})
        for x, y in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, -1)):
            frame = simulate({"r1": True, "x": x, "y": y, "right_x": .5, "right_y": -.5})
            self.assertEqual(frame["body_rotation"], (y * cfg.BODY_TILT_MAX, x * cfg.BODY_TILT_MAX, 0))
            self.assertEqual(frame["ramp"], 0)
            self.assertEqual(frame["phase"], 0)
            self.assertEqual(frame["turn"], 0)
            self.assertEqual(frame["body"]["size"], baseline["body"]["size"])
            for name, leg in frame["legs"].items():
                self.assert_point_equal(leg["target_world"], baseline["legs"][name]["target_world"])
                self.assert_point_equal(leg["points"][0], frame["origins"][name])
        roll = simulate({"r1": True, "y": 1})
        self.assertGreater(roll["origins"]["lf"][2], roll["origins"]["rf"][2])
        pitch = simulate({"r1": True, "x": 1})
        self.assertLess(pitch["origins"]["lf"][2], pitch["origins"]["lr"][2])
        self.assertLess(pitch["body"]["nose"][2], pitch["body"]["center"][2])
        self.assertEqual(simulate({}), baseline)

    def test_hardware_matches_simulator_and_returns_to_level(self):
        inputs = [(.7, -.5, True), (-.5, .7, True), (0, 0, True), (.4, .6, False)]
        frames = []
        with patch.object(main, "get_left_stick", side_effect=[(y, x) for x, y, _ in inputs] + [KeyboardInterrupt]), \
                patch.object(main, "get_right_stick", return_value=(.3, -.4)), \
                patch.object(main, "get_r1", side_effect=[r1 for _, _, r1 in inputs]), \
                patch.object(main, "get_dpad_vertical", return_value=0), \
                patch.object(main.time, "monotonic", side_effect=[i * .02 for i in range(6)]), \
                patch.object(main.time, "sleep"), patch("builtins.print"), \
                patch.object(main, "send_frame", side_effect=lambda sock, frame: frames.append(frame)):
            with self.assertRaises(KeyboardInterrupt):
                main.main_loop(Mock(), Mock())
        state = {}
        for hardware, (x, y, r1) in zip(frames[1:], inputs):
            preview = simulate({**state, "x": x, "y": y, "r1": r1, "right_x": .3, "right_y": -.4})
            state = {k: preview[k] for k in ("phase", "ramp", "direction", "turn")}
            self.assertEqual(hardware["body_rotation"], preview["body_rotation"])
            self.assertEqual(hardware["body_offset"], preview["body_offset"])
            for name, leg in hardware["legs"].items():
                self.assert_point_equal(leg["target"], preview["legs"][name]["target"])
                for joint, command in leg["servo_angles"].items():
                    self.assertAlmostEqual(command, preview["legs"][name]["servo_angles"][joint])
        self.assertEqual(frames[3]["body_rotation"], (0, 0, 0))
        self.assertEqual(frames[-1]["body_rotation"], (0, 0, 0))
        self.assertGreater(state["ramp"], 0)

    def test_entering_r1_stops_walking_and_turning(self):
        moving = simulate({"x": 1, "right_x": 1, "ramp": 1, "phase": .125})
        frame = simulate({**{k: moving[k] for k in ("phase", "ramp", "direction", "turn")},
                          "x": 1, "y": .5, "right_x": 1, "r1": True})
        self.assertLess(frame["ramp"], moving["ramp"])
        self.assertEqual(frame["turn"], 0)
        self.assertEqual(frame["direction"], moving["direction"])

    def test_unreachable_tilt_preserves_last_hardware_pose(self):
        frames = []
        with patch.object(cfg, "BODY_TILT_MAX", 8), \
                patch.object(main, "get_left_stick", side_effect=[(.5, 0), (1, 0), (0, 0), KeyboardInterrupt]), \
                patch.object(main, "get_right_stick", return_value=(0, 0)), \
                patch.object(main, "get_r1", return_value=True), \
                patch.object(main, "get_dpad_vertical", return_value=0), \
                patch.object(main.time, "sleep"), patch("builtins.print"), \
                patch.object(main, "send_frame", side_effect=lambda sock, frame: frames.append(frame)):
            real_step = main.step_motion

            def reject_full_tilt(*args, **kwargs):
                if kwargs.get("body_roll") == 8:
                    raise ValueError("unreachable pose")
                return real_step(*args, **kwargs)

            with patch.object(main, "step_motion", side_effect=reject_full_tilt):
                with self.assertRaises(KeyboardInterrupt):
                    main.main_loop(Mock(), Mock())
        self.assertEqual(frames[1]["body_rotation"], (4, 0, 0))
        self.assertEqual(frames[1], frames[2])
        self.assertEqual(frames[0], frames[3])


if __name__ == "__main__":
    unittest.main()
