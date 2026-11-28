"""Static stability: support polygon and ground projection of the CoM.

Pure geometry in robot axes (X forward, Y left, Z up); no dynamics. A foot
counts as a contact when its actual height (after servo limits) is within
CONTACT_TOLERANCE of the ground plane.
"""
import math

from . import config as cfg


def support_feet(feet, ground, tolerance=None):
    """Names of feet touching the ground, in the given dict order."""
    tolerance = cfg.CONTACT_TOLERANCE if tolerance is None else tolerance
    return [name for name, point in feet.items() if point[2] <= ground + tolerance]


def convex_hull(points):
    """Counter-clockwise hull without collinear points (monotone chain)."""
    points = sorted(set((float(x), float(y)) for x, y in points))
    if len(points) <= 2:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-9:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-9:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq < 1e-12 else max(0.0, min(1.0, (
        (p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def stability_margin(polygon, point):
    """Signed distance from point to the polygon boundary [mm].

    Positive inside, negative outside. A segment or single point has no
    interior, so the margin is minus the distance to it (0 when on it).
    Returns None without any contact.
    """
    if not polygon:
        return None
    if len(polygon) == 1:
        return -math.hypot(point[0] - polygon[0][0], point[1] - polygon[0][1])
    edges = list(zip(polygon, polygon[1:] + polygon[:1])) if len(polygon) > 2 \
        else [(polygon[0], polygon[1])]
    distance = min(_segment_distance(point, a, b) for a, b in edges)
    if len(polygon) == 2:
        return -distance
    inside = all((b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]) >= 0
                 for a, b in edges)
    return distance if inside else -distance


def stability_metric(settings):
    """Which number describes a gait: "line" when two legs swing at once.

    Then support collapses to a line (trot) and the distance from it matters;
    otherwise ("polygon", e.g. crawl) the margin inside the triangle does.
    """
    ratio = settings["swing_ratio"]
    offsets = list(settings["phase_offsets"].values())
    for i, a in enumerate(offsets):
        for b in offsets[i + 1:]:
            gap = (a - b) % 1.0
            if min(gap, 1.0 - gap) < ratio - 1e-9:
                return "line"
    return "polygon"


def support_state(feet, ground, com):
    """Contacts, support polygon and CoM margin for one frame.

    feet: {name: (x, y, z)} actual foot positions; com: (x, y, z) of the body.
    """
    contacts = support_feet(feet, ground)
    polygon = convex_hull([feet[name][:2] for name in contacts])
    projection = (com[0], com[1])
    margin = stability_margin(polygon, projection)
    kind = {0: "none", 1: "point", 2: "line"}.get(len(polygon), "polygon")
    return {"feet": contacts, "polygon": [list(p) for p in polygon], "kind": kind,
            "com": list(com), "com_ground": [com[0], com[1], ground],
            "margin": margin,
            "stable": kind == "polygon" and margin is not None and margin > 0,
            "warning_margin": cfg.STABILITY_WARNING_MARGIN}
