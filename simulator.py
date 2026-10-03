"""Local HTML simulator. Run: python simulator.py (no pygame required).
Pomarańczowy: osiągnięty limit serwa. Turkusowy pierścień: zadana pozycja stopy. 
Bryła korpusu jest przybliżona; mocowania pochodzą z LEG_ORIGINS.
Podgląd wszystkich 12 serw. main.py wysyła obecnie tylko serwa 1–6. 
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

ASSETS = Path(__file__).resolve().parent / "simulator"
PARAMETER_SPECS = {
    "gait_speed": {"min": 0, "max": 5, "step": 0.1, "unit": "Hz"},
    "step_length": {"min": 0, "max": 150, "step": 1, "unit": "mm"},
    "step_height": {"min": 0, "max": 100, "step": 1, "unit": "mm"},
    "z_height": {"min": -200, "max": 0, "step": 1, "unit": "mm"},
    "x_offset_front": {"min": -150, "max": 150, "step": 1, "unit": "mm"},
    "y_offset_front": {"min": 50, "max": 250, "step": 1, "unit": "mm"},
    "x_offset_rear": {"min": -150, "max": 150, "step": 1, "unit": "mm"},
    "y_offset_rear": {"min": 50, "max": 250, "step": 1, "unit": "mm"},
}


def parameter_specs():
    return {name: {**spec, "default": getattr(cfg, name)}
            for name, spec in PARAMETER_SPECS.items()}


def joint_points(name, angles):
    points = forward_kinematics(angles, cfg.l_coxa, cfg.l_femur, cfg.l_tibia)
    return [world_point(name, point) for point in points]


def world_point(name, point):
    origin = cfg.LEG_ORIGINS[name]
    side = 1 if name.startswith("l") else -1
    return [origin[0] + point[0], origin[1] + side * point[1], origin[2] + point[2]]


def simulate(data):
    parameters = {name: float(data.get(name, getattr(cfg, name)))
                  for name in PARAMETER_SPECS}
    for name, value in parameters.items():
        spec = PARAMETER_SPECS[name]
        if not math.isfinite(value) or not spec["min"] <= value <= spec["max"]:
            raise ValueError(f"{name}: wymagany zakres {spec['min']}–{spec['max']}")
    values = {key: float(data.get(key, default)) for key, default in
              (("phase", 0), ("ramp", 0), ("elapsed", 0.02), ("x", 0), ("y", 0))}
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError("Wartości muszą być skończone")
    if not (0 <= values["phase"] < 1 and 0 <= values["ramp"] <= 1
            and 0 <= values["elapsed"] <= 0.1 and abs(values["x"]) <= 1 and abs(values["y"]) <= 1):
        raise ValueError("Parametry poza zakresem")
    direction = tuple(data.get("direction", (0, 0)))
    if len(direction) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= 1 for v in direction):
        raise ValueError("Nieprawidłowy kierunek")
    state = {"phase": values["phase"], "ramp": values["ramp"], "direction": direction}
    # Omitted XY overrides retain each leg's configured starting position.
    motion_parameters = {name: value for name, value in parameters.items()
                         if "_offset_" not in name or name in data}
    state, frame = step_motion(state, values["x"], values["y"], values["elapsed"], **motion_parameters)
    for name, leg in frame["legs"].items():
        geometric, limited = [], []
        for joint, raw in zip(JOINT_NAMES, leg["raw_angles"]):
            spec = cfg.LEGS[name][joint]
            command = leg["servo_angles"][joint]
            limited.append(abs(command - apply_offsets(spec[cfg.SERVO_ID], raw)) > 1e-8)
            geometric.append(remove_offsets(spec[cfg.SERVO_ID], command))
        leg["points"] = joint_points(name, geometric)
        leg["target_world"] = world_point(name, leg["target"])
        leg["limited"] = limited
    frame.update(state)
    frame.update(origins=cfg.LEG_ORIGINS,
                 ground=parameters["z_height"])
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
