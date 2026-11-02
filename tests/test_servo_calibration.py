"""Calibration commands and cumulative trims, without hardware access."""

import io
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

from robot import config as cfg
from robot.servos import JOINTS
from tools import calibrate_servos as calibration


class ServoCalibrationTests(unittest.TestCase):
    def run_session(self, inputs):
        output = io.StringIO()
        with patch("builtins.input", side_effect=inputs), \
                patch.object(calibration, "send_servos") as send, \
                redirect_stdout(output):
            calibration.run(object())
        return send, output.getvalue()

    def test_starts_from_config_and_sends_absolute_angles(self):
        initial, baseline = calibration.starting_positions()
        send, output = self.run_session(["12 97", "4 135", "q"])
        self.assertEqual(send.call_args_list[0].args[1], initial)
        self.assertEqual(send.call_args_list[1].args[1], {12: 97})
        self.assertEqual(send.call_args_list[2].args[1], {4: 135})
        self.assertIn(f"12: {97 - baseline[12]:+.6f}".rstrip("0").rstrip("."), output)
        for sid in initial:
            self.assertIn(f"\n{sid}: ", output)

    def test_existing_trim_and_clipped_start_produce_replacement_trim(self):
        spec = JOINTS[1]
        for old_trim in (3.0, 200.0):
            with self.subTest(old_trim=old_trim), \
                    patch.dict(JOINTS, {1: (*spec[:cfg.TRIM], old_trim)}):
                initial, baseline = calibration.starting_positions()
                send, output = self.run_session(["1 100", "q"])
                self.assertEqual(send.call_args_list[0].args[1][1], initial[1])
                expected = 100 - baseline[1]
                self.assertIn(f"\n1: {expected:+.6f}".rstrip("0").rstrip("."), output)
                if old_trim == 200:
                    self.assertIn("Limit przyciął", output)

    def test_invalid_input_is_not_sent_and_limits_apply(self):
        send, output = self.run_session([
            "", "wrong", "13 90", "4 nan", "4 inf", "4 90 5 80", "4 999", "q",
        ])
        self.assertEqual(send.call_count, 2)
        self.assertEqual(send.call_args.args[1], {4: JOINTS[4][cfg.LIMITS][1]})
        self.assertIn("Nieprawidłowe dane", output)

    def test_inactive_servo_is_not_moved_and_keeps_trim(self):
        with patch.object(cfg, "ACTIVE_SERVO_IDS", {12}):
            send, output = self.run_session(["1 90", "q"])
        self.assertEqual(set(send.call_args.args[1]), {12})
        self.assertEqual(send.call_count, 1)
        self.assertIn(f"\n1: {JOINTS[1][cfg.TRIM]:+.6f}".rstrip("0").rstrip("."), output)

    def test_eof_and_ctrl_c_print_summary_without_resetting_pose(self):
        for stop in (EOFError, KeyboardInterrupt):
            with self.subTest(stop=stop):
                send, output = self.run_session(["12 97.5", stop])
                self.assertEqual(send.call_count, 2)
                self.assertEqual(send.call_args.args[1], {12: 97.5})
                self.assertIn("Pełne wartości TRIM", output)


if __name__ == "__main__":
    unittest.main()
