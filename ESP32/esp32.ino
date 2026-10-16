#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

// ====== TUTAJ WPISZ SWOJE DANE WIFI ======
const char* wifi_ssid     = "UPCF344A7E";
const char* wifi_password = "Hmu3zwsbycbm";
// ==========================================

// --- Ustawienia ---
#define DEBUG_PCA_PRINT           // zakomentuj tę linię, aby wyłączyć raport płytek przy starcie
#define DEBUG_SERVO_PRINT false   // true = wypisuje każde serwo (przy 50 Hz zalewa Serial i spowalnia pętlę)
#define SEND_REPLY        false   // true = odsyła "OK"/"ERR" (skrypt Pythona i tak nie czyta odpowiedzi)

// --- PCA9685 ---
const uint8_t I2C_SDA = 21;
const uint8_t I2C_SCL = 22;
const uint8_t PCA_ADDRS[] = {0x40, 0x41};
Adafruit_PWMServoDriver pca0(PCA_ADDRS[0]);
Adafruit_PWMServoDriver pca1(PCA_ADDRS[1]);
Adafruit_PWMServoDriver* const pcas[] = {&pca0, &pca1};
const uint8_t NUM_PCAS = sizeof(pcas) / sizeof(pcas[0]);
bool pcaReady[NUM_PCAS] = {};
bool pcaDetected[NUM_PCAS] = {};

void printPcaStatus() {
#ifdef DEBUG_PCA_PRINT
  for (uint8_t board = 0; board < NUM_PCAS; ++board) {
    const char* status = !pcaDetected[board] ? "brak odpowiedzi I2C (niepodłączona?)"
                        : pcaReady[board] ? "podłączona, gotowa"
                        : "podłączona, błąd inicjalizacji";
    Serial.printf("PCA9685 płytka %u, adres 0x%02X: %s\n",
                  board + 1, PCA_ADDRS[board], status);
  }
#endif
}

// --- UDP ---
WiFiUDP udp;
const int udpPort = 8888;

// --- Kalibracja ---
struct CalibrationPoint {
  float angle;
  int   pwm;   // mikrosekundy
};

// Jawny prototyp zapobiega wygenerowaniu go przez Arduino przed definicją typu.
int mapAngleToPWM(float angle, const CalibrationPoint points[3]);

struct ServoConfig {
  int id;
  uint8_t board;                   // 0 = 0x40, 1 = 0x41
  uint8_t channel;                 // kanał na PCA9685 (0-15)
  CalibrationPoint points[3];      // 0°, 90°, 180°
  float startAngle;
};

#define CAL_DEFAULT {{0.0, 600}, {90.0, 1500}, {180.0, 2400}}

// {ID, indeks plytki, kanal 0-15, kalibracja, kat startowy}
ServoConfig legServos[] = {
  { 1, 0, 1, {{0.0, 680}, {90.0, 1560}, {180.0, 2410}}, 90.0},   // noga 1: biodro
  { 2, 0, 2, {{0.0, 650}, {90.0, 1500}, {180.0, 2370}}, 153.0},  // noga 1: udo
  { 3, 0, 3, {{0.0, 690}, {90.0, 1530}, {180.0, 2380}}, 97.0},   // noga 1: goleń
  { 4, 0, 4, {{0.0, 650}, {90.0, 1510}, {180.0, 2350}}, 90.0},   // noga 2
  { 5, 0, 5, {{0.0, 650}, {90.0, 1480}, {180.0, 2350}}, 27.0},
  { 6, 0, 6, {{0.0, 700}, {90.0, 1560}, {180.0, 2410}}, 83.0},
  { 7, 1, 7, {{0.0, 600}, {90.0, 1460}, {180.0, 2300}}, 90.0},   // noga 3
  { 8, 1, 8, {{0.0, 670}, {90.0, 1510}, {180.0, 2340}}, 153.0},
  { 9, 1, 9, {{0.0, 700}, {90.0, 1560}, {180.0, 2400}}, 97.0},
  {10, 1, 10, {{0.0, 650}, {90.0, 1510}, {180.0, 2350}}, 90.0},   // noga 4: biodro
  {11, 1, 11, {{0.0, 625}, {90.0, 1480}, {180.0, 2330}}, 27.0},  // noga 4: udo
  {12, 1, 12, {{0.0, 600}, {90.0, 1440}, {180.0, 2280}}, 83.0}    // noga 4: goleń
};

const int numServos = sizeof(legServos) / sizeof(legServos[0]);

// --- Interpolacja kąt -> mikrosekundy ---
int mapAngleToPWM(float angle, const CalibrationPoint points[3]) {
  angle = constrain(angle, 0.0f, 180.0f);

  if (angle <= points[1].angle) {
    return points[0].pwm + (angle - points[0].angle) *
           (points[1].pwm - points[0].pwm) / (points[1].angle - points[0].angle);
  } else {
    return points[1].pwm + (angle - points[1].angle) *
           (points[2].pwm - points[1].pwm) / (points[2].angle - points[1].angle);
  }
}

// --- Wyślij szerokość impulsu na kanał PCA9685 ---
bool writeServoUs(uint8_t board, uint8_t channel, int us) {
  if (board >= NUM_PCAS || channel > 15 || !pcaReady[board]) return false;
  us = constrain(us, 400, 2600);
  pcas[board]->writeMicroseconds(channel, us);
  return true;
}

// --- Ustawienie serwa po ID ---
bool setServoById(int id, float angle) {
  for (int i = 0; i < numServos; i++) {
    if (legServos[i].id == id) {
      int us = mapAngleToPWM(angle, legServos[i].points);
      if (!writeServoUs(legServos[i].board, legServos[i].channel, us)) return false;
#if DEBUG_SERVO_PRINT
      Serial.printf("  Serwo %d (kanał %d) -> %.1f° (%d us)\n", id, legServos[i].channel, angle, us);
#endif
      return true;
    }
  }
  Serial.printf("  Nieznane ID serwa: %d\n", id);
  return false;
}

// --- Parsowanie JSON: {"set_servo": {"10": 90, "11": 45.5}} ---
bool parseAndExecuteCommand(const char* command) {
#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument doc;
#else
  DynamicJsonDocument doc(2048);
#endif

  DeserializationError error = deserializeJson(doc, command);
  if (error) {
    Serial.print("Błąd parsowania JSON: ");
    Serial.println(error.c_str());
    return false;
  }

  JsonObject cmd = doc["set_servo"];
  if (cmd.isNull()) return false;

  bool ok = true;
  for (JsonPair kv : cmd) {
    int id = atoi(kv.key().c_str());
    float angle = kv.value().as<float>();
    ok &= setServoById(id, angle);
  }
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nStart...");

  // --- PCA9685 ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);              // 400 kHz - przy 12 serwach na klatkę robi różnicę
  for (uint8_t board = 0; board < NUM_PCAS; ++board) {
    Wire.beginTransmission(PCA_ADDRS[board]);
    pcaDetected[board] = (Wire.endTransmission() == 0);
    if (!pcaDetected[board]) continue;
    pcaReady[board] = pcas[board]->begin();
    if (!pcaReady[board]) continue;
    pcas[board]->setOscillatorFrequency(27000000);
    pcas[board]->setPWMFreq(50);
  }
  printPcaStatus();
  delay(10);

  // Pozycje startowe - po kolei, żeby nie szarpnąć całym poborem prądu naraz
  for (int i = 0; i < numServos; i++) {
    if (!writeServoUs(legServos[i].board, legServos[i].channel,
                      mapAngleToPWM(legServos[i].startAngle, legServos[i].points))) {
      Serial.printf("Nie ustawiono serwa ID %d z płytki %u (kanał %u): sprawdź płytkę i kanał\n",
                    legServos[i].id, legServos[i].board + 1, legServos[i].channel);
      continue;
    }
    Serial.printf("Serwo ID %d z płytki %u na kanale %u ustawione na %.1f°\n",
                  legServos[i].id, legServos[i].board + 1,
                  legServos[i].channel, legServos[i].startAngle);
    delay(150);
  }

  // --- WiFi ---
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifi_ssid, wifi_password);

  Serial.printf("Łączenie z %s", wifi_ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("Połączono! IP ESP32: %s\n", WiFi.localIP().toString().c_str());

  // --- UDP ---
  udp.begin(udpPort);
  Serial.printf("UDP nasłuchuje na porcie %d\n", udpPort);
  Serial.println("System gotowy.");
}

void loop() {
  // Obsluz rowniez kolejne komendy czesciowe (sterowanie reczne).
  static char buf[1024];
  for (uint8_t packet = 0; packet < 8; ++packet) {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) break;
    IPAddress senderIP = udp.remoteIP();
    uint16_t senderPort = udp.remotePort();
    bool ok = false;
    if (packetSize >= static_cast<int>(sizeof(buf))) {
      udp.flush();
      Serial.println("Pakiet UDP za duzy");
    } else {
      int len = udp.read(buf, sizeof(buf) - 1);
      if (len == packetSize) {
        buf[len] = '\0';
        ok = parseAndExecuteCommand(buf);
      }
    }
#if SEND_REPLY
    udp.beginPacket(senderIP, senderPort);
    udp.write((const uint8_t*)(ok ? "OK" : "ERR"), ok ? 2 : 3);
    udp.endPacket();
#endif
  }

  // Co 10 s krótki raport o stanie WiFi
//   static unsigned long lastCheck = 0;
//   if (millis() - lastCheck > 10000) {
//     if (WiFi.status() == WL_CONNECTED)
//       Serial.printf("WiFi OK, IP: %s, sygnał: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
//     else
//       Serial.println("Brak połączenia z WiFi, próbuję wrócić...");
//     lastCheck = millis();
//   }

  delay(2);
}
