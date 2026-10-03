"""Regression checks for shared robot calculations and visualization geometry."""
import json
import math
import threading
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import config as cfg
from gait import calculate_trajectory, correct_angle
from gait import JOINTS
from ik import inverse_kinematics
from motion import calculate_frame, advance_motion, next_phase, JOINT_NAMES
from simulator import Handler, joint_points, world_point, simulate


class SimulatorTests(unittest.TestCase):
    def test_matches_original_robot_frame_and_fk(self):
        for direction in ((1, 0), (0, 1), (-1, 0), (0, -1), (.6, .8)):
            for phase in (0, .125, .25, .5, .75, .999):
                for ramp in (0, .4, 1):
                    frame = calculate_frame(phase, ramp, direction)
                    for name in cfg.LEGS:
                        target = calculate_trajectory(
                            (phase + cfg.LEG_PHASE_OFFSET[name]) % 1,
                            cfg.step_length * ramp, cfg.step_height * ramp,
                            cfg.p_start[name], *direction)
                        raw = inverse_kinematics(*target, cfg.l_coxa, cfg.l_femur, cfg.l_tibia)
                        for joint, angle in zip(JOINT_NAMES, raw):
                            sid = cfg.LEGS[name][joint][cfg.SERVO_ID]
                            self.assertEqual(frame["servos"][sid], correct_angle(sid, angle))
                        points = joint_points(name, raw)
                        for actual, expected in zip(points[-1], world_point(name, target)):
                            self.assertAlmostEqual(actual, expected, places=8)
                        for start, end, length in zip(points, points[1:],
                                                     (cfg.l_coxa, cfg.l_femur, cfg.l_tibia)):
                            self.assertAlmostEqual(math.dist(start, end), length)

    def test_ramp_and_limits(self):
        ramp = 0
        for _ in range(25):
            ramp = advance_motion(0, ramp, True, .02)
        self.assertAlmostEqual(ramp, 1)
        for _ in range(26):
            ramp = advance_motion(0, ramp, False, .02)
        self.assertEqual(ramp, 0)
        self.assertEqual(next_phase(.7, ramp, .02), 0)
        frame = simulate({})
        for name, leg in frame["legs"].items():
            self.assertFalse(leg["limited"][0])
            hip, coxa = leg["points"][:2]
            self.assertAlmostEqual(coxa[0], hip[0])
            self.assertAlmostEqual(coxa[1] - hip[1],
                                   cfg.l_coxa if name.startswith("l") else -cfg.l_coxa)
            self.assertAlmostEqual(math.dist(leg["points"][-1], leg["target_world"]), 0)
        # Force a servo limit to check that rendering still reflects clipping.
        spec = list(cfg.LEGS["lf"]["coxa"])
        spec[cfg.LIMITS] = (50, 70)
        with patch.dict(JOINTS, {spec[cfg.SERVO_ID]: tuple(spec)}):
            limited = simulate({})["legs"]["lf"]
        self.assertTrue(limited["limited"][0])
        self.assertGreater(math.dist(limited["points"][-1], limited["target_world"]), 1)
        with self.assertRaises(ValueError):
            simulate({"elapsed": float("nan")})

    def test_shifted_origins_translate_foot_and_target_together(self):
        before = simulate({})
        shifted = {name: (p[0] + 47, p[1] - 23, p[2] + 11)
                   for name, p in cfg.LEG_ORIGINS.items()}
        with patch.dict(cfg.LEG_ORIGINS, shifted):
            after = simulate({})
        for name in cfg.LEGS:
            self.assertAlmostEqual(math.dist(after["legs"][name]["target_world"],
                                            after["legs"][name]["points"][-1]), 0)
            for old, new in zip(before["legs"][name]["points"],
                                after["legs"][name]["points"]):
                for a, b, delta in zip(old, new, (47, -23, 11)):
                    self.assertAlmostEqual(b - a, delta)

    def test_http_page_and_frame(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            for path in ("/", "/sim.js", "/style.css"):
                with urlopen(base + path) as response:
                    self.assertEqual(response.status, 200)
                    self.assertTrue(response.read())
            request = Request(base + "/api/frame", data=json.dumps({"x": 1}).encode(),
                              headers={"Content-Type": "application/json"})
            with urlopen(request) as response:
                frame = json.load(response)
                self.assertEqual(len(frame["legs"]), 4)
                self.assertEqual(len(frame["servos"]), 12)
        finally:
            server.shutdown()
            server.server_close()
            worker.join()


if __name__ == "__main__":
    unittest.main()
