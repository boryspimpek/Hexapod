"""Local HTML simulator. Run: python simulator.py (no pygame required).
Pomarańczowy: osiągnięty limit serwa. Turkusowy pierścień: zadana pozycja stopy. 
Bryła korpusu jest przybliżona; mocowania pochodzą z LEG_ORIGINS.
Podgląd wszystkich 12 serw. main.py wysyła. 
Symulator nie łączy się z robotem.
Parametry chodu można zmieniać na żywo w panelu symulatora.
Model pokazuje kinematykę bez fizyki i kolizji."""

import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from robot import config as cfg
from robot.kinematics import forward_kinematics
from robot.servos import apply_offsets, remove_offsets
from robot.motion import step_motion, JOINT_NAMES
from robot.body import body_center, pose_point
from robot.odometry import read_odometry, advance_odometry

ASSETS = Path(__file__).resolve().parent / "simulator"
PARAMETER_SPECS = {
    "gait_speed": {"min": 0, "max": 5, "step": 0.1, "unit": "Hz"},
    "step_length": {"min": 0, "max": 150, "step": 1, "unit": "mm"},
    "step_height": {"min": 0, "max": 100, "step": 1, "unit": "mm"},
    "turn_step_angle": {"min": 0, "max": 30, "step": 0.5, "unit": "deg"},
    "z_height": {"min": -200, "max": 0, "step": 1, "unit": "mm"},
    "x_offset_front": {"min": -150, "max": 150, "step": 1, "unit": "mm"},
    "y_offset_front": {"min": 50, "max": 250, "step": 1, "unit": "mm"},
    "x_offset_rear": {"min": -150, "max": 150, "step": 1, "unit": "mm"},
    "y_offset_rear": {"min": 50, "max": 250, "step": 1, "unit": "mm"},
}


def parameter_specs():
    return {name: {**spec, "default": getattr(cfg, name)}
            for name, spec in PARAMETER_SPECS.items()}


def joint_points(name, angles, body_offset=(0, 0, 0), body_rotation=(0, 0, 0)):
    points = forward_kinematics(angles, cfg.l_coxa, cfg.l_femur, cfg.l_tibia,
                                mount_angle=cfg.COXA_MOUNT_ANGLES[name])
    return [world_point(name, point, body_offset, body_rotation) for point in points]


def world_point(name, point, body_offset=(0, 0, 0), body_rotation=(0, 0, 0)):
    origin = cfg.LEG_ORIGINS[name]
    side = 1 if name.startswith("l") else -1
    if body_rotation[0] or body_rotation[1]:
        local = (origin[0] + point[0], origin[1] + side * point[1], origin[2] + point[2])
        return list(pose_point(local, body_offset, *body_rotation[:2]))
    return [origin[0] + point[0] + body_offset[0],
            origin[1] + side * point[1] + body_offset[1],
            origin[2] + point[2] + body_offset[2]]


def simulate(data):
    odometry = read_odometry(data.get("odometry"))
    parameters = {name: float(data.get(name, getattr(cfg, name)))
                  for name in PARAMETER_SPECS}
    for name, value in parameters.items():
        spec = PARAMETER_SPECS[name]
        if not math.isfinite(value) or not spec["min"] <= value <= spec["max"]:
            raise ValueError(f"{name}: wymagany zakres {spec['min']}–{spec['max']}")
    values = {key: float(data.get(key, default)) for key, default in
              (("phase", 0), ("ramp", 0), ("elapsed", 0.02), ("x", 0), ("y", 0),
               ("right_x", 0), ("right_y", 0), ("turn", 0))}
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError("Wartości muszą być skończone")
    if not (0 <= values["phase"] < 1 and 0 <= values["ramp"] <= 1
            and 0 <= values["elapsed"] <= 0.1
            and all(abs(values[key]) <= 1 for key in ("x", "y", "right_x", "right_y", "turn"))):
        raise ValueError("Parametry poza zakresem")
    direction = tuple(data.get("direction", (0, 0)))
    if len(direction) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= 1 for v in direction):
        raise ValueError("Nieprawidłowy kierunek")
    shifting = data.get("r1", False)
    if not isinstance(shifting, bool):
        raise ValueError("r1: oczekiwano true/false")
    dpad = data.get("dpad", 0)
    previous_dpad = data.get("previous_dpad", 0)
    if any(type(value) not in (int, float) or value not in (-1, 0, 1)
           for value in (dpad, previous_dpad)):
        raise ValueError("D-pad: wymagane -1, 0 lub 1")
    original_height = parameters["z_height"]
    if dpad and dpad != previous_dpad:
        lo, hi = cfg.HEIGHT_LIMITS
        parameters["z_height"] = max(lo, min(hi, original_height - dpad * cfg.HEIGHT_STEP))
    state = {"phase": values["phase"], "ramp": values["ramp"], "direction": direction,
             "turn": 0.0 if shifting else values["turn"]}
    # Omitted XY overrides retain each leg's configured starting position.
    motion_parameters = {name: value for name, value in parameters.items()
                         if "_offset_" not in name or name in data}
    controls = dict(body_x=values["right_y"] * cfg.BODY_SHIFT_MAX if shifting else 0.0,
                    body_y=values["right_x"] * cfg.BODY_SHIFT_MAX if shifting else 0.0,
                    body_roll=values["y"] * cfg.BODY_TILT_MAX if shifting else 0.0,
                    body_pitch=values["x"] * cfg.BODY_TILT_MAX if shifting else 0.0,
                    turn=0.0 if shifting else values["right_x"])
    walk_x, walk_y = (0.0, 0.0) if shifting else (values["x"], values["y"])
    try:
        state, frame = step_motion(state, walk_x, walk_y, values["elapsed"],
                                   **controls, **motion_parameters)
    except ValueError:
        if parameters["z_height"] == original_height:
            raise
        parameters["z_height"] = motion_parameters["z_height"] = original_height
        state, frame = step_motion(state, walk_x, walk_y, values["elapsed"],
                                   **controls, **motion_parameters)
    body_offset = frame["body_offset"]
    body_rotation = frame["body_rotation"]
    for name, leg in frame["legs"].items():
        geometric, limited = [], []
        for joint, raw in zip(JOINT_NAMES, leg["raw_angles"]):
            spec = cfg.LEGS[name][joint]
            command = leg["servo_angles"][joint]
            limited.append(abs(command - apply_offsets(spec[cfg.SERVO_ID], raw)) > 1e-8)
            geometric.append(remove_offsets(spec[cfg.SERVO_ID], command))
        leg["points"] = joint_points(name, geometric, body_offset, body_rotation)
        leg["target_world"] = world_point(name, leg["target"], body_offset, body_rotation)
        leg["limited"] = limited
    frame.update(state)
    dimensions = tuple(max(p[i] for p in cfg.LEG_ORIGINS.values())
                       - min(p[i] for p in cfg.LEG_ORIGINS.values()) for i in range(2))
    center = body_center()
    frame["body"] = {
        "center": pose_point(center, body_offset, *body_rotation[:2]),
        "size": (dimensions[0] + 30, dimensions[1], 30),
        "nose": pose_point((center[0] + dimensions[0] / 2 + 15, center[1], center[2]),
                           body_offset, *body_rotation[:2]),
    }
    frame.update(origins={name: world_point(name, (0, 0, 0), body_offset, body_rotation)
                         for name in cfg.LEGS},
                 ground=parameters["z_height"], z_height=parameters["z_height"],
                 previous_dpad=dpad)
    frame["reference_center"] = center
    frame["odometry"] = advance_odometry(
        odometry, {name: leg["target_world"] for name, leg in frame["legs"].items()},
        values["phase"], moving=(parameters["gait_speed"] > 0
                                and (values["ramp"] > 0 or frame["ramp"] > 0)))
    return frame


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/api/parameters":
            self.respond(200, json.dumps(parameter_specs()).encode(), "application/json")
            return
        files = {"/": ("index.html", "text/html"), "/sim.js": ("sim.js", "text/javascript"),
                 "/style.css": ("style.css", "text/css")}
        asset = files.get(self.path)
        if not asset:
            self.send_error(404)
            return
        self.respond(200, (ASSETS / asset[0]).read_bytes(), asset[1])

    def do_POST(self):
        if self.path != "/api/frame":
            self.send_error(404)
            return
        try:
            size = int(self.headers.get("Content-Length", 0))
            if not 0 < size <= 4096:
                raise ValueError("Nieprawidłowy rozmiar żądania")
            data = json.loads(self.rfile.read(size))
            if not isinstance(data, dict):
                raise ValueError("Oczekiwano obiektu JSON")
            self.respond(200, json.dumps(simulate(data)).encode(), "application/json")
        except (ValueError, TypeError) as error:
            self.respond(400, json.dumps({"error": str(error)}).encode(), "application/json")

    def respond(self, status, content, mime):
        self.send_response(status)
        self.send_header("Content-Type", mime + "; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Symulator: http://127.0.0.1:{args.port} — Ctrl+C kończy pracę", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
