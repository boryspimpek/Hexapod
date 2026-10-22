"""Rigid body pose in robot axes: X forward, Y left, Z up (angles in degrees)."""
import math

from . import config as cfg


def body_center():
    return tuple((min(p[i] for p in cfg.LEG_ORIGINS.values())
                  + max(p[i] for p in cfg.LEG_ORIGINS.values())) / 2
                 for i in range(3))


def rotate_body(point, roll=0.0, pitch=0.0, *, inverse=False):
    """R = Ry(pitch) Rx(roll); positive angles lower right/front respectively."""
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    x, y, z = point
    if inverse:
        x, z = cp * x - sp * z, sp * x + cp * z
        return (x, cr * y + sr * z, -sr * y + cr * z)
    y, z = cr * y - sr * z, sr * y + cr * z
    return (cp * x + sp * z, y, -sp * x + cp * z)


def pose_point(point, offset=(0, 0, 0), roll=0.0, pitch=0.0, *, inverse=False):
    """Convert a body point to world coordinates, or world to body with inverse."""
    center = body_center()
    relative = tuple(point[i] - center[i] - (offset[i] if inverse else 0)
                     for i in range(3))
    rotated = rotate_body(relative, roll, pitch, inverse=inverse)
    return tuple(rotated[i] + center[i] + (0 if inverse else offset[i])
                 for i in range(3))
