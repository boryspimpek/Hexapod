"""Controller height changes without hardware or UDP."""
import unittest
from unittest.mock import Mock, patch

import main
from robot import config as cfg
from robot.joystick import get_dpad_vertical


class ControllerHeightTests(unittest.TestCase):
    def test_hat_and_button_fallback(self):
        controller = Mock()
        controller.get_numhats.return_value = 1
        for direction in (-1, 0, 1):
            controller.get_hat.return_value = (0, direction)
            self.assertEqual(get_dpad_vertical(controller), direction)
        controller.get_numhats.return_value = 0
        controller.get_numbuttons.return_value = 16
        controller.get_button.side_effect = lambda i: i == cfg.DPAD_UP_BUTTON
        self.assertEqual(get_dpad_vertical(controller), 1)
        controller.get_button.side_effect = lambda i: i == cfg.DPAD_DOWN_BUTTON
        self.assertEqual(get_dpad_vertical(controller), -1)
        controller.get_numbuttons.return_value = 0
        self.assertEqual(get_dpad_vertical(controller), 0)

    def run_loop(self, directions, height=-40):
        frames = []
        with patch.object(cfg, 'z_height', height), \
                patch.object(main, 'get_left_stick', return_value=(0, 0)), \
                patch.object(main, 'get_dpad_vertical', side_effect=[*directions, KeyboardInterrupt]), \
                patch.object(main, 'send_frame', side_effect=lambda sock, frame: frames.append(frame)), \
                patch.object(main.time, 'sleep'), patch('builtins.print'):
            with self.assertRaises(KeyboardInterrupt):
                main.main_loop(Mock(), Mock())
        return [frame['legs']['lf']['target'][2] for frame in frames]

    def test_press_hold_release_and_reverse(self):
        self.assertEqual(self.run_loop([1, 1, 0, 1, 0, -1]),
                         [-40, -45, -45, -45, -50, -50, -45])

    def test_height_limits(self):
        self.assertEqual(self.run_loop([1], cfg.HEIGHT_LIMITS[0]), [-150, -150])
        self.assertEqual(self.run_loop([-1], cfg.HEIGHT_LIMITS[1]), [-20, -20])


if __name__ == '__main__':
    unittest.main()
