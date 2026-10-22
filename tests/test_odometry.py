"""World travel from support targets, independent from physical robot control."""
import math
import unittest

from robot.body import body_center
from simulator import simulate


def world_foot(frame, name):
    point = frame["legs"][name]["target_world"]
    center = frame["reference_center"]
    pose = frame["odometry"]
    c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    x, y = point[0] - center[0], point[1] - center[1]
    return (pose["position"][0] + c * x - s * y,
            pose["position"][1] + s * x + c * y, point[2])


class OdometryTests(unittest.TestCase):
    def sequence(self, controls, count=100):
        state = {}
        frames = []
        for _ in range(count):
            frame = simulate({**state, **controls})
            frames.append(frame)
            state = {key: frame[key] for key in
                     ("phase", "ramp", "direction", "turn", "odometry")}
        return frames

    def assert_point_equal(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=7)

    def test_walk_directions_and_support_feet_stay_planted(self):
        for controls in ({"x": 1}, {"x": -1}, {"y": 1}, {"y": -1},
                         {"x": .6, "y": .8}, {"right_x": 1},
                         {"x": .6, "y": .8, "right_x": .5}):
            with self.subTest(controls=controls):
                frames = self.sequence(controls)
                for a, b in zip(frames, frames[1:]):
                    shared = a["odometry"]["contacts"].keys() & b["odometry"]["contacts"].keys()
                    for name in shared:
                        self.assert_point_equal(world_foot(a, name), world_foot(b, name))
                position = frames[-1]["odometry"]["position"]
                if "right_x" not in controls:
                    self.assertAlmostEqual(frames[-1]["odometry"]["yaw"], 0)
                    self.assertGreater((position[0] - body_center()[0]) * controls.get("x", 0)
                                       - (position[1] - body_center()[1]) * controls.get("y", 0), 50)
                elif "x" not in controls:
                    self.assertLess(frames[-1]["odometry"]["yaw"], -.1)
                    self.assert_point_equal(position, body_center()[:2])

    def test_heading_changes_walking_direction(self):
        first = simulate({"odometry": {"position": [40, 0], "yaw": math.pi / 2,
                                        "contacts": {}}})
        state = {"odometry": first["odometry"]}
        for _ in range(50):
            frame = simulate({**state, "x": 1})
            state = {k: frame[k] for k in ("phase", "ramp", "direction", "turn", "odometry")}
        self.assertAlmostEqual(frame["odometry"]["position"][0], 40)
        self.assertGreater(frame["odometry"]["position"][1], 30)

    def test_body_pose_does_not_create_world_travel(self):
        idle = simulate({})
        pose = idle["odometry"]
        for x, y, right_x, right_y in ((1, -.5, .7, -.4), (-1, .5, -.7, .4), (0, 0, 0, 0)):
            frame = simulate({"odometry": pose, "r1": True, "x": x, "y": y,
                              "right_x": right_x, "right_y": right_y})
            self.assert_point_equal(frame["odometry"]["position"], idle["odometry"]["position"])
            self.assertAlmostEqual(frame["odometry"]["yaw"], 0)
            pose = frame["odometry"]

    def test_stopping_zero_speed_zero_stride_and_reset(self):
        for settings in ({}, {"x": 1, "gait_speed": 0},
                         {"x": 1, "step_length": 0}):
            for frame in self.sequence(settings, 40):
                self.assert_point_equal(frame["odometry"]["position"], body_center()[:2])
        frames = self.sequence({"x": 1})
        state = {k: frames[-1][k] for k in ("phase", "ramp", "direction", "turn", "odometry")}
        for _ in range(30):
            frame = simulate(state)
            state = {k: frame[k] for k in state}
        position = frame["odometry"]["position"]
        for _ in range(10):
            frame = simulate(state)
            state = {k: frame[k] for k in state}
            self.assert_point_equal(frame["odometry"]["position"], position)
        reset = simulate({k: v for k, v in state.items() if k != "odometry"})
        self.assert_point_equal(reset["odometry"]["position"], body_center()[:2])
        self.assertEqual(reset["odometry"]["yaw"], 0)
        self.assertEqual(simulate({}), simulate({}))

    def test_invalid_state_is_rejected(self):
        valid = {"position": [40, 0], "yaw": 0, "contacts": {}}
        for value in ([], "bad", {}, {**valid, "yaw": float("nan")},
                      {**valid, "position": [float("inf"), 0]},
                      {**valid, "contacts": {"missing": [0, 0]}},
                      {**valid, "contacts": {"lf": [1]}},
                      {**valid, "yaw": 4}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                simulate({"odometry": value})

    def test_idle_parameter_changes_and_independent_sessions(self):
        traveling = self.sequence({"x": 1})[-1]
        before = simulate({})
        changed = simulate({"odometry": before["odometry"], "x_offset_front": 100,
                            "x_offset_rear": -100, "y_offset_front": 170})
        self.assertEqual(changed["odometry"]["position"], before["odometry"]["position"])
        self.assertNotEqual(traveling["odometry"]["position"], before["odometry"]["position"])
        self.assertEqual(simulate({}), before)


if __name__ == "__main__":
    unittest.main()
