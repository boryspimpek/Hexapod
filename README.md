# QUADRUPED MT 404 APEX

This repository contains the MT 404 Apex quadruped robot project, with inverse kinematics, gait generation, and a browser-based visualization. The code separates the robot logic from the simulator interface, allowing movement, servo calibration, and gait parameters to be tested without physical hardware.

![sim](<media/sim.png>)
![Overview](<media/overview.png>)

## HTML Simulator

Run `python simulator.py` from the project directory, then open
http://127.0.0.1:8000. To use a different port: `python simulator.py --port 8080`.
The server requires only the Python standard library; the browser loads
Three.js from a CDN, so opening the simulator for the first time requires Internet access.

Controls: WASD or the left gamepad stick for movement, the mouse to rotate the view,
and the scroll wheel to zoom.
The "Gait parameters" panel lets you adjust `gait_speed`,
`step_length`, `step_height`, and `z_height` live.
`z_height` sets a shared baseline.
The `x_offset_front`, `y_offset_front`, `x_offset_rear`, and `y_offset_rear`
sliders set the baseline foot coordinates for the front and rear leg pairs (in mm).
Configuration changes require restarting the server. JavaScript
only draws the received points and has no IK solver or gait generator of its own.

The body is represented by an approximate solid shape, and the mounting positions
come from `LEG_ORIGINS`.

## Inverse Kinematics

The `robot/kinematics.py` module determines the angles of three joints based on the target foot position `(x, y, z)`:

- **Coxa** rotates the leg in the X-Y plane.
- **Femur** and **Tibia** determine the leg position in the radial Y-Z plane.

The default segment lengths are defined in `robot/config.py`:

```python
l_coxa = 43.73
l_femur = 100.0
l_tibia = 149.10
```

## How Does Inverse Kinematics Work?

IK answers the question: **how should the three leg servos be positioned to place
the foot at a specified point?** For each leg separately (`LF` – left front,
`RF` – right front, `LR` – left rear, `RR` – right rear), the function receives
the foot position `(x, y, z)` in millimeters relative to the Coxa joint. It returns
the Coxa, Femur, and Tibia joint angles.

Coxa rotates the leg horizontally to point it toward the target (`X` and `Y`).
Femur and Tibia set its reach and height (`Y` and `Z`): Femur moves the upper
segment, while Tibia bends or straightens the lower segment. The axis directions
and angles are shown in the [top view](media/top_view_1.png) and
[front view](media/front_view_3.png).
If the point lies beyond the segments' reach, IK raises an error instead of
returning an unreachable configuration. The foot position is expressed in the
local coordinate system of each leg:

- `X` – direction along the robot's front-to-back axis,
- `Y` – direction outward from the robot; positive `Y` points outward for every leg,
- `Z` – vertical direction; positive `Z` points upward.

### Calculations

#### COXA

The COXA angle is calculated first, shown as `θ1` in the
[top view](media/top_view_1.png). The `atan2(x, y)` function determines the angle
of the foot position relative to the `+Y` axis using the target coordinates.
Since the servo takes an angle measured from its own zero position, the
`COXA_ZERO` offset (90° by default) is added to the result.

#### Maximum Reach

The leg's reach is checked next. First, we calculate the horizontal distance `r`
from the Coxa joint to the target, then subtract the Coxa segment length. This
gives `r'`, the horizontal distance from the end of Coxa to the target. Together
with the height `z`, it forms the segment `R` shown in the
[front view](media/front_view_3.png):

```text
r  = sqrt(x^2 + y^2)
r' = r - l_coxa
R  = sqrt(r'^2 + z^2)
```

The point is reachable when `R` lies between the difference and the sum of the
Femur and Tibia lengths: `abs(l_femur - l_tibia) <= R <= l_femur + l_tibia`.
Otherwise, the function raises an error.

#### TIBIA

The triangle's interior angle at the Tibia joint, labeled `θ3` in the
[front view](media/front_view_3.png), is calculated using the law of cosines.
The side lengths are `l_femur`, `l_tibia`, and `R`:

```text
theta_tibia = acos((l_femur^2 + l_tibia^2 - R^2)
                   / (2 * l_femur * l_tibia))
```

This is not yet the angle sent directly to the servo. The code converts it to
the Tibia mounting convention and expresses it in degrees:

```text
tibia_servo_angle = 180° - (degrees(theta_tibia) - TIBIA_OFFSET)
```

Because of Tibia's shape, `TIBIA_OFFSET` is added to align the foot tip with the
rest of the leg when the entire leg is horizontal. In addition, the Tibia angle
is measured in the opposite direction in the IK convention, using `180 - TIBIA`,
because the servo is mounted with a fixed horn while the servo body rotates.
This is illustrated in the [tibia offset diagram](media/tibia_offset.png).

#### FEMUR

The Femur angle consists of two parts. `α` describes the direction from the `-Z`
axis to the segment `R`, and `β` is the angle between `R` and Femur. Both are
shown in the [front view](media/front_view_3.png). We calculate `α` from the
target coordinates and `β` using the law of cosines:

```text
alpha = atan2(r', -z)
beta  = acos((l_femur^2 + R^2 - l_tibia^2)
             / (2 * l_femur * R))
theta_femur = degrees(alpha + beta)
```

The results of `atan2` and `acos` are in radians, so their sum is converted to
degrees. The resulting Femur angle and the corrected Tibia angle describe the
segment positions needed to place the foot at the target point.

## Axis Convention

- `X` - forward/backward direction; affects the coxa joint rotation,
- `Y` - leg extension direction; in the resting position, the leg points along `+Y`,
- `Z` - vertical direction, where positive values point upward.

Coordinates and lengths are given in millimeters, and angles are returned in degrees.

## Calculation Diagrams

### Coxa Angle and Radial Distance

![Diagram of the coxa angle and radial distance](<media/top_view_1.png>)

### Front View

![Diagram of the leg configuration below the Y-axis](<media/front_view_3.png>)

### Rear View

![Diagram of the leg configuration below the Y-axis](<media/rear_view_1.png>)

### Tibia Offset

![Tibia offset](<media/tibia_offset.png>)

### Servo Mounting Angles

This photo shows the initial servo position before assembling the leg structure.
Each servo should be set to 90 degrees in the configuration shown in the photo.

![Mounting angles](<media/mounting_angles.png>)

## Robot Configuration and Communication

The robot settings are in `robot/config.py`. The main groups are:

- **Geometry and gait:** `l_coxa`, `l_femur`, and `l_tibia` define the segment
  lengths in millimeters. `gait_speed`, `step_length`, `step_height`, and
  `ramp_time` control the step speed and shape. `LEG_PHASE_OFFSET` sets phase
  offsets between the legs, while `LEG_ORIGINS` defines their mounting positions
  used by the visualization.
- **Foot positions:** `p_start` contains the initial foot positions in each leg's
  local coordinate system. `x_offset_front`, `y_offset_front`, `x_offset_rear`,
  `y_offset_rear`, and `z_height` let you set these for the front and rear pairs.
- **Servos:** `LEGS` specifies each joint's servo ID, rotation direction
  (`inverted`), allowed angle range (`limits`), and mounting correction (`trim`).
  `robot/servos.py` applies inversion and trim, then clamps the angle to the
  specified range. Limits should match the mechanical range of the particular
  servo. `COXA_ZERO` and `TIBIA_OFFSET` correct angles according to the chosen
  convention and the way these joints are mounted.
- **Controls:** `stick_deadzone` sets the stick dead zone, while
  `CONTROLLER_DEADZONE` sets the movement control dead zone. `HEIGHT_STEP` and
  `HEIGHT_LIMITS` define the body height increment and range; `DPAD_UP_BUTTON`
  and `DPAD_DOWN_BUTTON` are fallback D-pad button indices.
- **Communication:** `ESP` contains the receiver's IP address and port, while
  `ACTIVE_SERVO_IDS` selects the servos that receive commands.
  `LOOP_INTERVAL` defines the interval between control loop iterations.

`main.py` sends commands to the ESP over UDP. `robot/transport.py` encodes them
as JSON, for example `{"set_servo": {"1": 90.0}}`; the robot program rounds angles
to 0.1 degrees. The ESP receiver must be configured to listen at the address
specified in `ESP` and understand this format. The HTML simulator does not send
commands to the robot. Configuration changes take effect after restarting the
robot program or the simulator server.

## Code Organization

- `main.py`: hardware startup and the robot control loop.
- `robot/config.py`: geometry, gait parameters, calibration, ESP address, and active servos.
- `robot/gait.py`: foot trajectories and direction normalization.
- `robot/kinematics.py`: IK and FK using the same angle convention.
- `robot/servos.py`: inversion, trim, angle limiting, and angle reconstruction for FK.
- `robot/motion.py`: frame calculation and the shared motion step for the robot and simulator.
- `robot/joystick.py`: functions for opening, reading, and closing the gamepad.
- `robot/transport.py`: UDP socket and the shared ESP command format.
- `simulator.py`: API validation, preview preparation, and the HTTP server.
- `simulator/`: HTML, CSS, and JavaScript for the preview.
- `tools/`: helper tools.
- `archive/`: older versions.

The motion state is a plain dictionary with the fields `phase`, `ramp`, and
`direction`. `step_motion(state, x, y, elapsed)` returns a new state and a frame
without modifying the input dictionary. Importing modules does not open the
gamepad or a socket. Resources are closed when the program exits. The only
custom class in the application code is the handler required by the standard
HTTP server.

Run these commands from the project's root directory:

```sh
python main.py
python simulator.py
python -m tools.manual_servos
python tools/print_angles.py
python -m robot.joystick
```

Robot control and gamepad diagnostics require `pygame`.
Manual control accepts final servo angles: it applies limits without reapplying
inversion or trim. The robot program rounds angles to 0.1 degrees; the manual
tool preserves the supplied precision.
