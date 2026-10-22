# MT 404 "APEX" - QUADRUPED ROBOT

This repository contains the MT 404 Apex quadruped robot project, with inverse kinematics, gait generation, and a browser-based visualization. The robot is controlled with a gamepad, and servo commands are sent to the ESP over UDP. The code separates the robot logic from the simulator interface, allowing movement, servo calibration, and gait parameters to be tested without physical hardware.

![sim](<media/sim.png>)
![Overview](<media/overview.png>)

## HTML Simulator

Run `python simulator.py` from the project directory, then open
http://127.0.0.1:8000. To use a different port: `python simulator.py --port 8080`.
The server requires only the Python standard library; the browser loads
Three.js from a CDN, so opening the simulator for the first time requires Internet access.

Controls: WASD or the left gamepad stick for movement, the mouse to rotate the view,
and the scroll wheel to zoom.
The right gamepad stick's horizontal axis turns the robot left/right by stepping
along arcs around the body's center. Its vertical axis is ignored in this mode.
Turning works in place or together with walking from the left stick.
`turn_step_angle` sets the yaw stride in degrees at full deflection (10 degrees
by default); smaller deflections reduce the rotation. Start/stop uses the gait ramp.
Hold **R1** while moving the right stick to shift the body forward/backward and
left/right relative to the feet instead. Deflection sets the body position
directly; releasing the stick or R1 returns the body to center.
In the simulator, left/right arrows turn; **Shift + arrows** shifts the body.
Hold **R1 + left stick** to tilt the body: right lowers its right side (roll),
forward lowers its front (pitch). Deflection sets the angle directly, up to
`BODY_TILT_MAX` degrees per axis (default 8°). Releasing the stick or R1 levels
the body. In the simulator use **Shift + WASD**. While R1 is held, the left stick
controls tilt instead of walking; an existing gait ramps down. Both sticks can
be used together to combine tilt and body translation. The inverse body pose
compensates all three foot coordinates, keeping stationary foot targets fixed
on the ground. During gait ramp-down, feet finish their gait motion.
The simulator shows servo-limit clipping in orange; fixed foot targets are
achieved by the physical legs only within their mechanical and servo limits.

The simulator also tracks the robot's position and heading on the ground.
Consecutive targets of legs in stance determine a planar translation and
rotation, so walking moves the robot across the grid and turning changes its
heading. Subsequent joystick directions follow the robot's current heading.
Each new support pair establishes fresh ground contacts. This is kinematic
odometry based on commanded targets; it does not model friction, slipping,
collisions, or balance, and servo clipping can make the displayed feet diverge
from their targets. Body shifts and tilts under R1 retain the ground targets.
**Follow robot** toggles camera tracking while preserving camera rotation,
zoom, and pan. **Reset position** returns the robot's world position and heading
to their starting values. Each browser tab has its own travel state; reloading
the page resets it. This visualization does not change physical robot commands.
D-pad up/down (or up/down arrows without Shift in the simulator) changes
`z_height` by `HEIGHT_STEP` per press within `HEIGHT_LIMITS`, like on the robot.
Holding a direction does not repeat the change; release and press again.
`BODY_SHIFT_MAX` sets the maximum translation per axis in millimeters (20 mm
by default). This translation is added independently of the baseline foot
offsets and can also be used while walking.
All hardware stick axis indices are in `robot/config.py`:
`LEFT_STICK_X_AXIS` and `LEFT_STICK_Y_AXIS` (0 and 1 by default), and
`RIGHT_STICK_X_AXIS` and `RIGHT_STICK_Y_AXIS` (2 and 3 by default).
The browser uses the standard gamepad mapping (left axes 0/1, right axes 2/3).
`R1_BUTTON` sets the hardware button index (10, verified on this controller). If your driver uses
a different mapping, identify R1 using `python -m robot.joystick` and change
this setting. The browser uses standard gamepad button 5 for R1.
The "Gait parameters" panel lets you adjust `gait_speed`,
`step_length`, `step_height`, `turn_step_angle`, and `z_height` live.
`z_height` sets a shared baseline.
The `x_offset_front`, `y_offset_front`, `x_offset_rear`, and `y_offset_rear`
sliders set the baseline foot coordinates for the front and rear leg pairs (in mm).
Configuration changes require restarting the server. JavaScript
only draws the received points and has no IK solver or gait generator of its own.

The body is represented by an approximate solid shape, and the mounting positions
come from `LEG_ORIGINS`. These positions also determine the turning center and
each foot's rotation radius, so they should match the robot's geometry.

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

The motion state is a plain dictionary with the fields `phase`, `ramp`,
`direction`, and `turn`. `step_motion(state, x, y, elapsed, turn=0)` returns a new state and a frame
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


## Two PCA9685 boards (ESP32)

`esp32.ino` supports two boards on the same I2C bus. Keep the first at
`0x40`; bridge the **A0 address pads** on the second to select `0x41`.
Connect both SDA pins to ESP32 GPIO 21, both SCL pins to GPIO 22,
both VCC pins to 3.3 V, and connect all grounds together. Connect servo
power to V+ using an external supply suitable for the servos; do not
power the servos from the ESP32 3.3 V pin.

The default wiring is:

| Servo IDs | PCA9685 address | Channels, in ID order |
| --- | --- | --- |
| 1-6 | 0x40 | 0-5 |
| 7-12 | 0x41 | 0-5 |

Each row in `legServos` contains `{id, board, channel, calibration, startAngle}`.
Board `0` selects `PCA_ADDRS[0]`, board `1` selects `PCA_ADDRS[1]`.
Change this table to match your wiring; the existing per-servo pulse calibration
and starting angles are preserved. Startup prints which boards were found.

Python keeps sending logical IDs, for example
`{"set_servo":{"1":90,"7":120}}` selects channel 0 on each board.
No Python protocol changes are needed. Set `ESP` in `robot/config.py` to the
IP printed by the ESP32. Run `python -m tools.manual_servos` to test one ID
at a time within its configured limits. With `SEND_REPLY` set to `false`,
the manual tool reports no reply even when the command was executed;
set it to `true` for OK/ERR replies during manual tests.
