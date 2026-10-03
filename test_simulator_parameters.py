"""Checks for live gait settings and independent simulator sessions."""
import unittest
from unittest.mock import patch

from robot import config as cfg
from robot.motion import initial_state, step_motion
from simulator import simulate, parameter_specs


class GaitParameterTests(unittest.TestCase):
    def test_live_settings_change_phase_and_trajectory(self):
        frame = simulate({"x": 1, "phase": .125, "ramp": 1,
                          "gait_speed": 2, "step_length": 80, "step_height": 20})
        self.assertAlmostEqual(frame["phase"], .165)
        target = frame["legs"]["rf"]["target"]
        self.assertAlmostEqual(target[0], cfg.p_start["rf"][0] - 20)
        self.assertAlmostEqual(target[2], cfg.p_start["rf"][2] + 20 * 2 ** -.5)

    def test_zero_settings_and_session_isolation(self):
        request = {"x": 1, "phase": .125, "ramp": 1}
        before = simulate(request)
        stopped = simulate({**request, "gait_speed": 0, "step_length": 0, "step_height": 0})
        self.assertEqual(stopped["phase"], .125)
        for name, leg in stopped["legs"].items():
            self.assertEqual(leg["target"], cfg.p_start[name])
        self.assertEqual(simulate(request), before)
        state, frame = step_motion(initial_state(), 1, 0, .02)
        default = simulate({"x": 1})
        self.assertEqual(default["phase"], state["phase"])
        self.assertEqual(default["legs"]["rf"]["target"], frame["legs"]["rf"]["target"])

    def test_rejects_invalid_parameters(self):
        for name, maximum in (("gait_speed", 5), ("step_length", 150), ("step_height", 100)):
            for value in (-1, maximum + 1, float("nan"), float("inf"), "invalid"):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    simulate({name: value})

    def test_common_height_preserves_individual_xy_and_sessions(self):
        starts = {name: (i * 5, 150 + i * 4, cfg.z_height)
                  for i, name in enumerate(cfg.LEGS)}
        with patch.dict(cfg.p_start, starts):
            before = simulate({})
            frame = simulate({"z_height": -80})
            self.assertEqual(frame["ground"], -80)
            for name, leg in frame["legs"].items():
                self.assertEqual(leg["target"], (*starts[name][:2], -80))
                self.assertEqual(leg["target_world"][2], cfg.LEG_ORIGINS[name][2] - 80)
            self.assertEqual(simulate({}), before)
            self.assertEqual(cfg.p_start, starts)

    def test_height_default_and_swing(self):
        with patch.object(cfg, "z_height", -65):
            self.assertEqual(parameter_specs()["z_height"]["default"], -65)
            self.assertEqual(simulate({})["ground"], -65)
            _, frame = step_motion(initial_state(), 0, 0, .02)
            for leg in frame["legs"].values():
                self.assertEqual(leg["target"][2], -65)
        frame = simulate({"z_height": -80, "phase": .25, "ramp": 1,
                          "x": 1, "step_height": 20})
        self.assertAlmostEqual(frame["legs"]["rf"]["target"][2], -60)
        self.assertAlmostEqual(frame["legs"]["lf"]["target"][2], -80)
        for value in (-201, 1, float("nan"), float("inf"), "invalid"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                simulate({"z_height": value})


if __name__ == "__main__":
    unittest.main()
