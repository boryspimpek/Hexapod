"""Support polygon, contact detection and CoM stability margin."""
import unittest

from robot.stability import (convex_hull, stability_margin, stability_metric,
                             support_feet, support_state)
from simulator import simulate

SQUARE = [(0, 0), (100, 0), (100, 100), (0, 100)]


class GeometryTests(unittest.TestCase):
    def test_hull_is_ccw_and_drops_inner_and_collinear_points(self):
        hull = convex_hull([(0, 0), (100, 100), (50, 50), (100, 0), (0, 100), (50, 0)])
        self.assertEqual(hull, [(0, 0), (100, 0), (100, 100), (0, 100)])

    def test_margin_sign_inside_outside_and_on_edge(self):
        self.assertAlmostEqual(stability_margin(SQUARE, (50, 30)), 30)
        self.assertAlmostEqual(stability_margin(SQUARE, (130, 50)), -30)
        self.assertAlmostEqual(stability_margin(SQUARE, (100, 50)), 0)
        self.assertAlmostEqual(stability_margin(SQUARE, (103, 104)), -5)

    def test_triangle_line_point_and_empty(self):
        triangle = convex_hull([(0, 0), (100, 0), (0, 100)])
        self.assertAlmostEqual(stability_margin(triangle, (10, 20)), 10)
        self.assertAlmostEqual(stability_margin([(0, 0), (100, 100)], (50, 50)), 0)
        self.assertAlmostEqual(stability_margin([(0, 0), (100, 0)], (50, 7)), -7)
        self.assertAlmostEqual(stability_margin([(0, 0)], (3, 4)), -5)
        self.assertIsNone(stability_margin([], (0, 0)))

    def test_contact_uses_actual_height_with_tolerance(self):
        feet = {"lf": (0, 0, -50), "rf": (0, 0, -48.5), "lr": (0, 0, -40), "rr": (0, 0, -55)}
        self.assertEqual(support_feet(feet, -50, tolerance=2), ["lf", "rf", "rr"])

    def test_support_state_kinds(self):
        feet = {"lf": (100, 100, 0), "rf": (100, 0, 0), "lr": (0, 100, 0), "rr": (0, 0, 30)}
        state = support_state(feet, 0, (60, 60, 50))
        self.assertEqual(state["kind"], "polygon")
        self.assertEqual(state["feet"], ["lf", "rf", "lr"])
        self.assertTrue(state["stable"])
        self.assertEqual(state["com_ground"], [60, 60, 0])
        state = support_state({**feet, "lr": (0, 100, 30)}, 0, (60, 60, 50))
        self.assertEqual(state["kind"], "line")
        self.assertFalse(state["stable"])


class SimulatorSupportTests(unittest.TestCase):
    def test_metric_follows_overlapping_swings(self):
        self.assertEqual(simulate({"gait": "trot"})["support"]["metric"], "line")
        self.assertEqual(simulate({"gait": "crawl", "requested_gait": "crawl"})["support"]["metric"],
                         "polygon")
        self.assertEqual(stability_metric({"swing_ratio": .2, "phase_offsets":
                                           {"lf": 0, "rf": .5, "lr": .75, "rr": .25}}), "polygon")
        self.assertEqual(stability_metric({"swing_ratio": .3, "phase_offsets":
                                           {"lf": 0, "rf": .5, "lr": .75, "rr": .25}}), "line")

    def test_standing_has_four_feet_and_centered_com(self):
        support = simulate({})["support"]
        self.assertEqual(sorted(support["feet"]), ["lf", "lr", "rf", "rr"])
        self.assertEqual(support["kind"], "polygon")
        self.assertGreater(support["margin"], 0)

    def test_trot_swing_leaves_diagonal_line(self):
        support = simulate({"gait": "trot", "x": 1, "ramp": 1, "phase": .15})["support"]
        self.assertEqual(sorted(support["feet"]), ["lr", "rf"])
        self.assertEqual(support["kind"], "line")

    def test_crawl_swing_leaves_stable_triangle(self):
        support = simulate({"gait": "crawl", "requested_gait": "crawl",
                            "x": 1, "ramp": 1, "phase": .1})["support"]
        self.assertEqual(sorted(support["feet"]), ["lr", "rf", "rr"])
        self.assertTrue(support["stable"])


if __name__ == "__main__":
    unittest.main()
