# Hexapod

The project contains inverse kinematics (IK) calculations for a three-segment hexapod leg and a visualization of its movement.

## Inverse Kinematics

The `ik.py` module determines the angles of three joints based on the target foot position `(x, y, z)`:

- **Coxa** rotates the leg in the X-Y plane.
- **Femur** and **Tibia** determine the leg position in the radial Y-Z plane.

The default segment lengths are defined in `config.py`:

```python
l_coxa = 40.0
l_femur = 80.0
l_tibia = 120.0
```

## Axis Convention

- `X` - forward/backward direction; affects the coxa joint rotation,
- `Y` - leg extension direction; in the resting position, the leg points along `+Y`,
- `Z` - vertical direction, where positive values point upward.

Coordinates and lengths are given in millimeters, and angles are returned in degrees.

## Calculation Diagrams

### Coxa Angle and Radial Distance

![Diagram of the coxa angle and radial distance](<media/Zrzut ekranu 2026-09-11 193820.png>)

### Leg Configuration Above the Y-Axis

![Diagram of the leg configuration above the Y-axis](<media/Zrzut ekranu 2026-09-11 193857.png>)

### Leg Configuration Below the Y-Axis

![Diagram of the leg configuration below the Y-axis](<media/below.png>)

