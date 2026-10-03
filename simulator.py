"""Local HTML simulator. Run: python simulator.py (no pygame required)."""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import config as cfg
from gait import apply_offsets, calc_dir
from motion import calculate_frame, advance_motion, next_phase, JOINT_NAMES

ASSETS = Path(__file__).resolve().parent / "simulator"


def joint_points(name, angles):
    """FK for rendering"""
    coxa = math.radians(angles[0] - 90)
    femur, tibia = map(math.radians, angles[1:])
    radial = (math.sin(coxa), math.cos(coxa))
    knee_direction = femur + tibia + math.radians(25) - math.pi
    a = (cfg.l_coxa * radial[0], cfg.l_coxa * radial[1], 0)
    b = (a[0] + cfg.l_femur * math.sin(femur) * radial[0],
         a[1] + cfg.l_femur * math.sin(femur) * radial[1],
         -cfg.l_femur * math.cos(femur))
    c = (b[0] + cfg.l_tibia * math.sin(knee_direction) * radial[0],
         b[1] + cfg.l_tibia * math.sin(knee_direction) * radial[1],
         b[2] - cfg.l_tibia * math.cos(knee_direction))
    return [world_point(name, p) for p in ((0, 0, 0), a, b, c)]


def world_point(name, point):
    origin = cfg.LEG_ORIGINS[name]
    side = 1 if name.startswith("l") else -1
    return [origin[0] + point[0], origin[1] + side * point[1], origin[2] + point[2]]


def simulate(data):
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
    running = math.hypot(values["x"], values["y"]) > cfg.stick_deadzone
    if running:
        direction = calc_dir(values["x"], values["y"])
    phase = values["phase"]
    ramp = advance_motion(phase, values["ramp"], running, values["elapsed"])
    frame = calculate_frame(phase, ramp, direction)
    for name, leg in frame["legs"].items():
        geometric, limited = [], []
        for joint, raw in zip(JOINT_NAMES, leg["raw_angles"]):
            spec = cfg.LEGS[name][joint]
            command = leg["servo_angles"][joint]
            limited.append(abs(command - apply_offsets(spec[cfg.SERVO_ID], raw)) > 1e-8)
            angle = command - spec[cfg.TRIM]
            geometric.append(180 - angle if spec[cfg.INVERTED] else angle)
        leg["points"] = joint_points(name, geometric)
        leg["target_world"] = world_point(name, leg["target"])
        leg["limited"] = limited
    frame.update(phase=next_phase(phase, ramp, values["elapsed"]), ramp=ramp,
                 direction=direction, origins=cfg.LEG_ORIGINS,
                 ground=min(p[2] for p in cfg.p_start.values()))
    return frame


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
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
