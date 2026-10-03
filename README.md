# Hexapod

## Symulator HTML korzystający z logiki robota

Uruchom z katalogu projektu: `python simulator.py`, następnie otwórz
http://127.0.0.1:8000. Inny port: `python simulator.py --port 8080`.
Serwer wymaga tylko biblioteki standardowej Pythona; przeglądarka pobiera
Three.js z CDN, więc pierwsze otwarcie wymaga Internetu.

Sterowanie: WASD lub lewy drążek pada, mysz do obrotu i kółko do zoomu.
Pauza zatrzymuje fazę, a pozycja spoczynkowa zeruje stan chodu.
`motion.py` jest wspólnym źródłem obliczeń dla `main.py` i `simulator.py`:
korzysta bezpośrednio z `config.py`, `gait.py` i `ik.py`. Zmiany konfiguracji
wymagają ponownego uruchomienia serwera. JavaScript tylko rysuje otrzymane
punkty, bez własnego IK i generatora chodu. Stary `x_SIM` pozostaje niezależny.

Korpus jest przybliżoną bryłą, mocowania wynikają z `LEG_ORIGINS`.
Podgląd pokazuje wszystkie cztery nogi i kąty 12 serw po inwersji, trimie
i limitach. Pomarańczowe oznaczenia sygnalizują ograniczenia, a turkusowe
pierścienie pokazują zadane pozycje stóp. Geometria po ograniczeniach jest
odtwarzana przez kinematykę prostą w Pythonie, z konwencją montażu z `ik.py`.
Nie jest to symulacja fizyki, kontaktu z podłożem ani kolizji.
`main.py` nadal wysyła tylko `ACTIVE_SERVO_IDS` (obecnie 1–6).
Symulator nie importuje sterownika pada ani nie wysyła UDP.
Każda karta ma własny stan; obliczenia używają kroku 20 ms, więc przy wolnych
odpowiedziach animacja zwalnia zamiast pomijać klatki.

Sprawdzenie zgodności obliczeń: `python -m unittest test_simulator.py`.

The project contains inverse kinematics (IK) calculations for a three-segment quadruped leg and a visualization of its movement.

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

![Diagram of the coxa angle and radial distance](<media/3.png>)

### Leg Configuration

![Diagram of the leg configuration below the Y-axis](<media/4.png>)

