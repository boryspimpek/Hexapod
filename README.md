# QUADRUPED MT 404 APEX

To repozytorium zawiera projekt czteronożnego robota quadruped MT 404 Apex z implementacją kinematyki odwrotnej, generowania chodu oraz wizualizacji działania w przeglądarce. Kod oddziela logikę robota od interfejsu symulatora, dzięki czemu można testować ruch, kalibrację serw i parametry chodu bez bezpośredniego sprzętu.

## Symulator HTML

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

## Jak działa kinematyka odwrotna?

IK odpowiada na pytanie: **jak ustawić trzy serwa nogi, żeby stopa znalazła się
w zadanym punkcie?** Dla każdej nogi osobno (`LF` – lewa przednia, `RF` – prawa
przednia, `LR` – lewa tylna, `RR` – prawa tylna) funkcja dostaje pozycję stopy
`(x, y, z)` w milimetrach, liczoną względem stawu Coxa. Zwraca kąty stawów
Coxa, Femur i Tibia.

Coxa obraca nogę w poziomie, aby skierować ją w stronę punktu (`X` i `Y`).
Femur i Tibia ustawiają jej wysięg oraz wysokość (`Y` i `Z`): Femur porusza
górnym segmentem, a Tibia zgina lub prostuje dolny. Kierunki osi i kąty pokazują
[widok z góry](media/top_view_1.png) oraz [widok z przodu](media/front_view_3.png).
Jeśli punkt leży poza zasięgiem segmentów, IK zgłasza błąd zamiast zwracać
nieosiągalne ustawienie. Pozycję stopy opisujemy w lokalnym układzie
współrzędnych danej nogi:

- `X` – kierunek wzdłuż osi przód–tył robota,
- `Y` – kierunek na zewnątrz robota; dodatnie `Y` wskazuje na zewnątrz dla każdej nogi,
- `Z` – kierunek pionowy; dodatnie `Z` wskazuje w górę.


### Obliczenia

#### COXA

W pierwszej kolejności obliczany jest kąt COXA, widoczny jako `θ1` na
[widoku z góry](media/top_view_1.png). Funkcja `atan2(x, y)` wyznacza kąt
położenia stopy względem osi `+Y`, korzystając ze współrzędnych jej celu.
Ponieważ serwo przyjmuje kąt liczony od własnego położenia zerowego, do wyniku
dodawane jest przesunięcie `COXA_ZERO` (domyślnie 90°).

#### Max reach

Następnie sprawdzany jest zasięg nogi. Najpierw obliczamy poziomy zasięg od
stawu Coxa do celu `r`, a potem odejmujemy długość segmentu Coxa. Otrzymujemy
`r'`, czyli poziomą odległość od końca Coxa do celu. Wraz z wysokością `z`
tworzy ona odcinek `R` widoczny na [widoku z przodu](media/front_view_3.png):

```text
r  = sqrt(x^2 + y^2)
r' = r - l_coxa
R  = sqrt(r'^2 + z^2)
```

Punkt jest osiągalny, gdy `R` mieści się między różnicą i sumą długości Femura
i Tibii: `abs(l_femur - l_tibia) <= R <= l_femur + l_tibia`. W przeciwnym razie
funkcja zgłasza błąd.

#### TIBIA

Kąt wewnętrzny trójkąta w stawie Tibia, na rysunku [widoku z
przodu](media/front_view_3.png) oznaczony jako `θ3` i wyznaczamy z twierdzenia cosinusów.
Jego boki mają długości `l_femur`, `l_tibia` i `R`:

```text
theta_tibia = acos((l_femur^2 + l_tibia^2 - R^2)
				   / (2 * l_femur * l_tibia))
```

To jeszcze nie jest bezpośrednio kąt wysyłany do serwa. W kodzie jest on
przeliczany na konwencję montażu Tibii i wyrażony w stopniach:

```text
kat_serwa_tibii = 180° - (degrees(theta_tibia) - TIBIA_OFFSET)
```
Tibia, ze względu na swój kształt ma dodany `TIBIA_OFFSET`, aby końcówka stopy przy ustawiniu poziomym całej nogi była w lini. Dodatkowo, w konwencji IK kąt Tibia jest liczony w przeciwnym kierunku, czyli `180 - TIBIA` ponieważ serwo zamontowane jest tak, że orczyk jest sztywny, a obraca się serwo. Uwidocznione jest to na rysunku [tibia offset](media/tibia_offset.png) 

#### FEMUR

Kąt Femura składa się z dwóch części. `α` określa kierunek od osi `-Z` do
odcinka `R`, a `β` jest kątem między `R` i Femurem. Oba widać na [widoku z
przodu](media/front_view_3.png). `α` obliczamy ze współrzędnych celu, a `β`
z twierdzenia cosinusów:

```text
alpha = atan2(r', -z)
beta  = acos((l_femur^2 + R^2 - l_tibia^2)
			 / (2 * l_femur * R))
theta_femur = degrees(alpha + beta)
```

Wyniki `atan2` i `acos` są w radianach, dlatego ich suma jest zamieniana na
stopnie. Otrzymany kąt Femura oraz skorygowany kąt Tibii opisują ustawienie
segmentów tak, aby stopa znalazła się w zadanym punkcie.

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

### TIBIA offset

![Tibia offset](<media/tibia_offset.png>)

###

To zdjecie pokazuje wstępne ustawienie serwa przed montażem konstrukcji nogi. Każde serwo powinno mieć ustawione 90 stopni w konfiguracji pokazanej na zdjęciu.

![Mounting angels](<media/mounting_angles.png>)

## Konfiguracja robota i komunikacja

Ustawienia robota znajdują się w `robot/config.py`. Najważniejsze grupy:

- **Geometria i chód:** `l_coxa`, `l_femur` i `l_tibia` określają długości
	segmentów w milimetrach. `gait_speed`, `step_length`, `step_height` i
	`ramp_time` sterują tempem oraz kształtem kroku. `LEG_PHASE_OFFSET` ustawia
	przesunięcia faz między nogami, a `LEG_ORIGINS` określa położenie ich mocowań
	używane przez wizualizację.
- **Pozycje stóp:** `p_start` zawiera pozycje początkowe stóp w lokalnym układzie
	każdej nogi. Wartości `x_offset_front`, `y_offset_front`, `x_offset_rear`,
	`y_offset_rear` i `z_height` pozwalają ustawić je dla przedniej i tylnej pary.
- **Serwa:** w `LEGS` dla każdego stawu podaje się ID serwa, kierunek obrotu
	(`inverted`), zakres dozwolonych kątów (`limits`) i korektę montażową (`trim`).
	`robot/servos.py` stosuje inwersję i trim, a następnie ogranicza kąt do podanego
	zakresu. Limity należy dobrać do mechanicznego zakresu konkretnego serwa.
	`COXA_ZERO` i `TIBIA_OFFSET` korygują kąty wynikające z przyjętej konwencji
	oraz sposobu montażu tych stawów.
- **Sterowanie:** `stick_deadzone` ustawia martwą strefę drążka, a
	`CONTROLLER_DEADZONE` martwą strefę sterowania ruchem. `HEIGHT_STEP` i
	`HEIGHT_LIMITS` określają zmianę i zakres wysokości korpusu; `DPAD_UP_BUTTON`
	oraz `DPAD_DOWN_BUTTON` są zapasowymi numerami przycisków D-pada.
- **Komunikacja:** `ESP` zawiera adres IP i port odbiornika, a
	`ACTIVE_SERVO_IDS` wybiera serwa, do których wysyłane są komendy.
	`LOOP_INTERVAL` określa odstęp między iteracjami pętli sterowania.

Program `main.py` wysyła komendy do ESP przez UDP. `robot/transport.py` koduje
je jako JSON, na przykład `{"set_servo": {"1": 90.0}}`; kąty są w programie
robota zaokrąglane do 0,1 stopnia. Odbiornik ESP musi być skonfigurowany tak, by
nasłuchiwał pod adresem z `ESP` i rozumiał ten format. Symulator HTML nie wysyła
komend do robota. Zmiany konfiguracji zastosuj po ponownym uruchomieniu
programu robota lub serwera symulatora.

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
