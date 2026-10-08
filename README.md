# Hexapod

Sterowanie wysokoscia w `python main.py`: D-pad gora podnosi korpus
(`z_height` maleje), D-pad dol obniza korpus. Kazde nacisniecie zmienia
wysokosc o 5 mm; przytrzymanie nie powtarza zmiany. Dziala w postoju i podczas
chodu. Ustawienie obowiazuje do zakonczenia programu.
Krok i zakres (-150 do -20 mm) ustawiaja `HEIGHT_STEP` i `HEIGHT_LIMITS`
w `robot/config.py`. Zmiana poza zasieg IK jest odrzucana; limity serw nadal
obowiazuja. Diagnostyka `python -m robot.joystick` pokazuje D-pad i numery
przyciskow. Gdy pad nie udostepnia hat, dostosuj `DPAD_UP_BUTTON` i
`DPAD_DOWN_BUTTON` do numerow z diagnostyki.

## Symulator HTML korzystający z logiki robota

Uruchom z katalogu projektu: `python simulator.py`, następnie otwórz
http://127.0.0.1:8000. Inny port: `python simulator.py --port 8080`.
Serwer wymaga tylko biblioteki standardowej Pythona; przeglądarka pobiera
Three.js z CDN, więc pierwsze otwarcie wymaga Internetu.

Sterowanie: WASD lub lewy drążek pada, mysz do obrotu i kółko do zoomu.
Panel „Gait parameters” pozwala zmieniać na żywo `gait_speed` (0–5 Hz),
`step_length` (0–150 mm), `step_height` (0–100 mm) i `z_height` (−200–0 mm).
`z_height` ustawia wspólne bazowe Z stóp, zachowując indywidualne X i Y nóg;
bardziej ujemna wartość zwiększa wysokość korpusu nad podłożem. Ustawienia są osobne
dla każdej karty, a odświeżenie przywraca wartości z `robot/config.py`.
Suwaki `x_offset_front`, `y_offset_front`, `x_offset_rear` i `y_offset_rear`
ustawiają bazowe współrzędne stóp przedniej i tylnej pary nóg (w mm).
Zakres X: −150–150 mm; Y: 50–250 mm. Dodatnie X przesuwa stopy do przodu,
a większe Y odsuwa je na zewnątrz po obu stronach robota.
Pauza zatrzymuje fazę, a pozycja spoczynkowa zeruje stan chodu.
`robot/motion.py` jest wspólnym źródłem obliczeń dla `main.py` i `simulator.py`:
korzysta z konfiguracji, trajektorii, kinematyki i kalibracji serw w pakiecie `robot`. Zmiany konfiguracji
wymagają ponownego uruchomienia serwera. JavaScript tylko rysuje otrzymane
punkty, bez własnego IK i generatora chodu. Stare wersje są przechowywane w `archive/`.

Korpus jest przybliżoną bryłą, mocowania wynikają z `LEG_ORIGINS`.
Podgląd pokazuje wszystkie cztery nogi i kąty 12 serw po inwersji, trimie
i limitach. Pomarańczowe oznaczenia sygnalizują ograniczenia, a turkusowe
pierścienie pokazują zadane pozycje stóp. Geometria po ograniczeniach jest
odtwarzana przez kinematykę prostą w Pythonie, z konwencją montażu z `robot/kinematics.py`.
Nie jest to symulacja fizyki, kontaktu z podłożem ani kolizji.
`main.py` nadal wysyła tylko `ACTIVE_SERVO_IDS` (obecnie 1–6).
Symulator nie importuje sterownika pada ani nie wysyła UDP.
Każda karta ma własny stan; obliczenia używają kroku 20 ms, więc przy wolnych
odpowiedziach animacja zwalnia zamiast pomijać klatki.


The project contains inverse kinematics (IK) calculations for a three-segment quadruped leg and a visualization of its movement.

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

## Axis Convention

- `X` - forward/backward direction; affects the coxa joint rotation,
- `Y` - leg extension direction; in the resting position, the leg points along `+Y`,
- `Z` - vertical direction, where positive values point upward.

Coordinates and lengths are given in millimeters, and angles are returned in degrees.

## Calculation Diagrams

### Coxa Angle and Radial Distance

![Diagram of the coxa angle and radial distance](<media/top_view_1.png>)

### Front view

![Diagram of the leg configuration below the Y-axis](<media/front_view_3.png>)

### Rear view

![Diagram of the leg configuration below the Y-axis](<media/rear_view_1.png>)

## Organizacja kodu

- `main.py`: uruchomienie sprzętu i pętla sterowania robotem.
- `robot/config.py`: geometria, parametry chodu, kalibracja, adres ESP i aktywne serwa.
- `robot/gait.py`: trajektorie stóp i normalizacja kierunku.
- `robot/kinematics.py`: IK i FK w tej samej konwencji kątów.
- `robot/servos.py`: inwersja, trim, ograniczanie kątów i odtwarzanie kątów do FK.
- `robot/motion.py`: obliczanie klatki i wspólny krok ruchu dla robota i symulatora.
- `robot/joystick.py`: otwieranie, odczyt i zamykanie pada przez funkcje.
- `robot/transport.py`: socket UDP i wspólny format komend ESP.
- `simulator.py`: walidacja API, przygotowanie podglądu i serwer HTTP.
- `simulator/`: HTML, CSS i JavaScript podglądu.
- `tools/`: narzędzia pomocnicze; 
- `archive/`: stare wersje.

Stan ruchu jest zwykłym słownikiem z polami `phase`, `ramp`, `direction`.
`step_motion(state, x, y, elapsed)` zwraca nowy stan i klatkę bez zmiany wejściowego
słownika. Import modułów nie otwiera pada ani socketu. Zasoby są zamykane po
zakończeniu programu. Jedyna własna klasa w kodzie aplikacji to handler wymagany
przez standardowy serwer HTTP.

Uruchamiaj z głównego katalogu projektu:

```sh
python main.py
python simulator.py
python -m tools.manual_servos
python tools/print_angles.py
python -m robot.joystick
```

Sterowanie robotem i diagnostyka pada wymagają `pygame`.
Ręczne sterowanie przyjmuje końcowe kąty serw: stosuje limity, bez ponownego
nakładania inwersji i trimu. Program robota zachowuje zaokrąglenie do 0,1 stopnia;
narzędzie ręczne zachowuje podaną precyzję.
