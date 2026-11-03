"""Simulator-only planar odometry fitted to consecutive stance-foot targets.

This follows the commanded kinematics, without simulating friction or dynamics.
Each browser carries its own pose and previous support samples.
"""
import math

from . import config as cfg
from .body import body_center
from .gait import gait_settings


def read_odometry(value):
    center = body_center()
    if value is None:
        return {"position": list(center[:2]), "yaw": 0.0, "contacts": {}}
    if not isinstance(value, dict):
        raise ValueError("Nieprawidłowy stan odometrii")
    position, yaw, contacts = value.get("position"), value.get("yaw"), value.get("contacts")
    def vector(point, size):
        return (isinstance(point, (list, tuple)) and len(point) == size
                and all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 1e9
                        for v in point))
    if (not vector(position, 2) or type(yaw) not in (int, float)
            or not math.isfinite(yaw) or abs(yaw) > math.pi + 1e-9
            or not isinstance(contacts, dict)
            or any(name not in cfg.LEGS or not vector(point, 2)
                   for name, point in contacts.items())):
        raise ValueError("Nieprawidłowy stan odometrii")
    return {"position": list(position), "yaw": yaw,
            "contacts": {name: list(point) for name, point in contacts.items()}}


def advance_odometry(previous, feet, phase, *, moving=True, gait=None, turn=0.0):
    """Estimate pose change from shared stance targets.

    At support exchange there are no shared anchors: preserve the pose and
    establish the newly landed feet. Straight walking estimates translation
    only, since changing stride length during ramp-up deforms the support
    geometry; rotation is fitted only while a turn is commanded. Servo
    clipping is deliberately excluded.
    """
    center = body_center()
    settings = gait_settings(gait)
    contacts = {name: [point[0] - center[0], point[1] - center[1]]
                for name, point in feet.items()
                if (phase + settings["phase_offsets"][name]) % 1.0 >= settings["swing_ratio"]}
    shared = sorted(contacts.keys() & previous["contacts"].keys())
    position, yaw = list(previous["position"]), previous["yaw"]
    if shared and moving:
        old = [previous["contacts"][name] for name in shared]
        new = [contacts[name] for name in shared]
        a = [sum(p[i] for p in new) / len(new) for i in range(2)]
        b = [sum(p[i] for p in old) / len(old) for i in range(2)]
        dot = sum((p[0] - a[0]) * (q[0] - b[0]) + (p[1] - a[1]) * (q[1] - b[1])
                  for p, q in zip(new, old))
        cross = sum((p[0] - a[0]) * (q[1] - b[1]) - (p[1] - a[1]) * (q[0] - b[0])
                    for p, q in zip(new, old))
        # During straight walking, ramping the stride length changes the
        # support-foot geometry. A free SE(2) fit mistakes that deformation
        # for body rotation, especially with crawl's three support legs.
        # Straight commands have no commanded heading change, so estimate
        # translation from the support centroid and reserve rotation fitting
        # for an active turn.
        delta = (math.atan2(cross, dot)
                 if abs(turn) > cfg.stick_deadzone and math.hypot(dot, cross) > 1e-12
                 else 0.0)
        c, s = math.cos(delta), math.sin(delta)
        dx, dy = b[0] - (c * a[0] - s * a[1]), b[1] - (s * a[0] + c * a[1])
        c, s = math.cos(yaw), math.sin(yaw)
        position[0] += c * dx - s * dy
        position[1] += s * dx + c * dy
        yaw = (yaw + delta + math.pi) % (2 * math.pi) - math.pi
    return {"position": position, "yaw": yaw, "contacts": contacts}
