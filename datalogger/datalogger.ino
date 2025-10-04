/*******************************************************
 * Datalogger IoT ESP32 + YL-83 + RTC DS1307 + microSD + LCD + MQTT
 * Requisitos cubiertos:
 * - JSON: fecha, hora, lectura, estado actuador (con timestamps ON/OFF)
 * - MQTT (broker público Mosquitto) topic: "20194989_Pedro"
 * - LCD I2C + Serial + MQTT simultáneo
 * - microSD: archivo diario AAAA_MM_DD.json, append cada 10 s
 * - Tareas con millis() (sin delay): sensor(5 s), sd(10 s), pantalla, mqtt, actuador, LED
 * - Clases y métodos, documentación en código
 ********************************************************/

// ===================== Librerías =====================
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <RTClib.h>              // Adafruit RTClib (DS1307)
#include <LiquidCrystal_I2C.h>   // LCD I2C (LiquidCrystal_I2C)
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

// ===================== Configuración =====================
// ---- WiFi ----
const char* WIFI_SSID = "Spartanplay0.0";
const char* WIFI_PASS = "halo2004";

// ---- MQTT (broker público Mosquitto) ----
const char* MQTT_HOST = "test.mosquitto.org";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP32_YL83_Datalogger_20194989";
const char* MQTT_TOPIC = "20194989_Pedro";

// ---- Pines (ajusta según tu hardware) ----
const int PIN_YL83_ADC = 34;     // Entrada AO del YL-83 a GPIO34 (ADC1, solo entrada)
const int PIN_BUZZER   = 25;     // Buzzer (idealmente con transistor si es de 5V)
const int PIN_SD_CS    = 5;      // Chip Select del módulo microSD (SPI)
const int PIN_LED_R    = 15;     // LED RGB - Rojo
const int PIN_LED_G    = 2;      // LED RGB - Verde (en muchos ESP32 es LED on-board)
const int PIN_LED_B    = 4;      // LED RGB - Azul

// ---- LCD I2C ----
const uint8_t LCD_ADDR = 0x27;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ---- Umbrales del sensor (calibra en tu equipo) ----
// Lectura cruda BAJA = mojado, ALTA = seco (histéresis)
int UMBRAL_ON  = 1700;   // <= activa buzzer (mojado)
int UMBRAL_OFF = 1900;   // >= apaga buzzer (seco)

// ---- Periodos (ms) ----
const uint32_t PERIODO_SENSOR_MS   = 5000;   // i. tarea_sensor (5 s)
const uint32_t PERIODO_SD_MS       = 10000;  // iv. tarea_sd (10 s)
const uint32_t PERIODO_PANTALLA_MS = 500;    // refresco pantalla
const uint32_t PERIODO_MQTT_MS     = 2000;   // ciclo MQTT
const uint32_t PERIODO_LED_MS      = 300;    // parpadeo status

// ===================== Objetos globales =====================
WiFiClient espClient;
PubSubClient mqtt(espClient);
RTC_DS1307 rtc;

// Estado global del sistema
struct ActuatorState {
  bool active = false;
  String last_on  = "";  // "YYYY-MM-DD HH:MM:SS"
  String last_off = "";
};

struct DataPacket {
  String date;        // "DD-MM-YYYY"
  String time;        // "HH:MM:SS"
  int    sensorRaw;   // lectura ADC
  ActuatorState actu;
};

// ===================== Utilidades de tiempo/fecha =====================
String twoDigits(int v) { return (v < 10) ? "0" + String(v) : String(v); }

String fmtDate_DDMMYYYY(const DateTime& dt) {
  return twoDigits(dt.day()) + "-" + twoDigits(dt.month()) + "-" + String(dt.year());
}
String fmtDateFilename_AAAAMMDD(const DateTime& dt) {
  return String(dt.year()) + "_" + twoDigits(dt.month()) + "_" + twoDigits(dt.day());
}
String fmtTime(const DateTime& dt) {
  return twoDigits(dt.hour()) + ":" + twoDigits(dt.minute()) + ":" + twoDigits(dt.second());
}
String fmtTimestamp(const DateTime& dt) {
  return String(dt.year()) + "-" + twoDigits(dt.month()) + "-" + twoDigits(dt.day()) +
         " " + fmtTime(dt);
}

// ===================== Clases de módulos =====================

// ---- Sensor YL-83 (ADC) ----
class SensorYL83 {
public:
  explicit SensorYL83(int pin) : _pin(pin), _lastValue(0) {}
  void begin() { pinMode(_pin, INPUT); }
  int readRaw() { _lastValue = analogRead(_pin); return _lastValue; }
  int last() const { return _lastValue; }
private:
  int _pin;
  int _lastValue;
};

// ---- Actuador (Buzzer) con parpadeo no bloqueante y registro ON/OFF ----
class ActuatorBuzzer {
public:
  explicit ActuatorBuzzer(int pin) : _pin(pin) {}
  void begin() { pinMode(_pin, OUTPUT); off(); }
  bool isOn() const { return _on; }

  // Estado lógico (se llama al cruzar umbrales)
  void on(const DateTime& now) {
    if (!_on) {
      _on = true;
      _lastOnStamp = fmtTimestamp(now);
      // iniciar pulso inmediatamente en HIGH
      _hwState = true;
      digitalWrite(_pin, HIGH);
      _lastToggle = millis();
    }
  }
  void off(const DateTime& now) {
    if (_on) {
      _on = false;
      _lastOffStamp = fmtTimestamp(now);
    }
    _hwState = false;
    digitalWrite(_pin, LOW);
  }
  void off() { _on = false; _hwState = false; digitalWrite(_pin, LOW); }

  // Tick de pulso (llamar en loop): ON 150ms / OFF 250ms por defecto
  void pulseTick(uint32_t onMs = 150, uint32_t offMs = 250) {
    if (!_on) { // aseguramos salida LOW cuando está inactivo
      if (_hwState) { _hwState = false; digitalWrite(_pin, LOW); }
      return;
    }
    uint32_t now = millis();
    uint32_t intervalo = _hwState ? onMs : offMs;
    if (now - _lastToggle >= intervalo) {
      _hwState = !_hwState;
      digitalWrite(_pin, _hwState ? HIGH : LOW);
      _lastToggle = now;
    }
  }

  const String& lastOn()  const { return _lastOnStamp; }
  const String& lastOff() const { return _lastOffStamp; }

private:
  int _pin;
  bool _on = false;          // estado lógico (mojado = activo)
  bool _hwState = false;     // salida física HIGH/LOW para el parpadeo
  uint32_t _lastToggle = 0;
  String _lastOnStamp = "";
  String _lastOffStamp = "";
};

// ---- LCD I2C 16x2 ----
class DisplayLCD {
public:
  void begin() {
    lcd.init();
    lcd.backlight();
    splash();
  }
  void show(const String& line1, const String& line2) {
    lcd.setCursor(0,0); lcd.print(pad(line1));
    lcd.setCursor(0,1); lcd.print(pad(line2));
  }
  void splash() {
    lcd.clear();
    show("ESP32 Datalogger", "YL-83 + MQTT + SD");
    delay(800);
    lcd.clear();
  }
private:
  String pad(const String& s) {
    String t = s;
    if (t.length() < 16) {
      t.reserve(16);
      while (t.length() < 16) t += ' ';
    }
    return t.substring(0, 16);
  }
};

// ---- microSD (append JSON por día) ----
class StorageSD {
public:
  bool begin(int csPin) {
    _ok = SD.begin(csPin);
    return _ok;
  }
  bool ok() const { return _ok; }

  bool appendJSON(const DateTime& now, const String& jsonLine) {
    if (!_ok) return false;
    const String fname = fmtDateFilename_AAAAMMDD(now) + ".json";
    File f = SD.open("/" + fname, FILE_APPEND);
    if (!f) return false;
    bool w = (f.println(jsonLine), true);
    f.close();
    return w;
  }
private:
  bool _ok = false;
};

// ---- MQTT ----
class MqttClient {
public:
  void begin(const char* host, uint16_t port) { mqtt.setServer(host, port); }
  bool ensureConnected() {
    if (mqtt.connected()) return true;
    String cid = String(MQTT_CLIENT_ID) + "_" + String((uint32_t)esp_random(), HEX);
    return mqtt.connect(cid.c_str());
  }
  bool publish(const char* topic, const String& payload) {
    if (!mqtt.connected()) return false;
    return mqtt.publish(topic, payload.c_str());
  }
  void loop() { mqtt.loop(); }
  bool connected() const { return mqtt.connected(); }
};

// ---- LED RGB de estado ----
// Estados solicitados:
// - Rojo sólido: sin internet (Wi-Fi no conectado)
// - Azul parpadeando: Wi-Fi conectado pero MQTT no (conectando/buscando)
// - Verde sólido: Wi-Fi + MQTT OK
enum NetState { NET_NO_INTERNET, NET_CONNECTING, NET_MQTT_OK };

class LedRGB {
public:
  LedRGB(int r, int g, int b): R(r), G(g), B(b) {}
  void begin() { pinMode(R, OUTPUT); pinMode(G, OUTPUT); pinMode(B, OUTPUT); off(); }
  void set(NetState s) { state = s; }
  void tick() {
    uint32_t now = millis();
    if (now - last >= PERIODO_LED_MS) { last = now; phase = !phase; }
    switch (state) {
      case NET_NO_INTERNET: setColor(1, 0, 0); break;           // rojo sólido
      case NET_CONNECTING:  setColor(0, 0, phase ? 1:0); break; // azul parpadeo
      case NET_MQTT_OK:     setColor(0, 1, 0); break;           // verde sólido
    }
  }
  void off() { setColor(0,0,0); }
private:
  void setColor(int r, int g, int b) {
    digitalWrite(R, r ? HIGH : LOW);
    digitalWrite(G, g ? HIGH : LOW);
    digitalWrite(B, b ? HIGH : LOW);
  }
  int R, G, B;
  NetState state = NET_NO_INTERNET;
  uint32_t last = 0; bool phase = false;
};

// ===================== Instancias =====================
SensorYL83     sensor(PIN_YL83_ADC);
ActuatorBuzzer buzzer(PIN_BUZZER);
DisplayLCD     screen;
StorageSD      sdcard;
MqttClient     mq;
LedRGB         led(PIN_LED_R, PIN_LED_G, PIN_LED_B);

// ===================== Timers de tareas =====================
uint32_t t_sensor=0, t_sd=0, t_pantalla=0, t_mqtt=0;

// Datos actuales
DataPacket current;

// ===================== Setup/Utilidades de conectividad =====================
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando a WiFi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-start < 15000) {
    Serial.print(".");
    delay(250);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi no disponible (timeout).");
  }
}

void ensureRTC() {
  if (!rtc.begin()) {
    Serial.println("RTC no encontrado. Verifica conexiones.");
  }
  if (!rtc.isrunning()) {
    // Si no corre, setea con fecha/hora de compilación (una sola vez)
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

// ===================== Serializa JSON =====================
String buildJSON(const DataPacket& d) {
  StaticJsonDocument<512> doc;
  doc["date"] = d.date;      // "DD-MM-YYYY"
  doc["time"] = d.time;      // "HH:MM:SS"
  doc["sensor"] = d.sensorRaw;

  JsonObject actu = doc.createNestedObject("actuator");
  actu["active"] = d.actu.active;
  if (d.actu.last_on.length())  actu["last_on"]  = d.actu.last_on;   // "YYYY-MM-DD HH:MM:SS"
  if (d.actu.last_off.length()) actu["last_off"] = d.actu.last_off;

  String out;
  serializeJson(doc, out);
  return out;
}

// ===================== Tareas =====================

// i. tarea_sensor: cada 5 s actualizar sensor
void tarea_sensor() {
  if (millis() - t_sensor < PERIODO_SENSOR_MS) return;
  t_sensor = millis();

  int raw = sensor.readRaw();
  current.sensorRaw = raw;
}

// ii. tarea_actu: lógica de activación según umbral + histéresis
void tarea_actu() {
  DateTime now = rtc.now();

  // ACTIVAR cuando esté MOJADO (lectura baja)
  if (!buzzer.isOn() && current.sensorRaw <= UMBRAL_ON) {
    buzzer.on(now);
  }
  // APAGAR cuando suba al rango SECO (lectura alta)
  else if (buzzer.isOn() && current.sensorRaw >= UMBRAL_OFF) {
    buzzer.off(now);
  }

  current.actu.active   = buzzer.isOn();
  current.actu.last_on  = buzzer.lastOn();
  current.actu.last_off = buzzer.lastOff();
}

// iii. tarea_pantalla: fecha/hora, valor, mensaje de activación
void tarea_pantalla() {
  if (millis() - t_pantalla < PERIODO_PANTALLA_MS) return;
  t_pantalla = millis();

  DateTime now = rtc.now();
  current.date = fmtDate_DDMMYYYY(now);
  current.time = fmtTime(now);

  String l1 = current.date + " " + current.time; // 16c cortará automáticamente
  String l2 = "YL83:" + String(current.sensorRaw);
  if (current.actu.active) l2 = "Buzzer ACTIVO!";
  screen.show(l1, l2);
}

// iv. tarea_sd: cada 10 s, guarda JSON en archivo AAAA_MM_DD.json
void tarea_sd() {
  if (millis() - t_sd < PERIODO_SD_MS) return;
  t_sd = millis();

  DateTime now = rtc.now();
  String json = buildJSON(current);
  bool w = sdcard.appendJSON(now, json);
  Serial.println(w ? String("[SD] OK -> ") + json
                   : String("[SD] ERROR de escritura"));
}

// v. tarea_mqtt: transmitir info al broker
void tarea_mqtt() {
  if (millis() - t_mqtt < PERIODO_MQTT_MS) return;
  t_mqtt = millis();

  // Reconexión automática
  if (WiFi.status() == WL_CONNECTED) {
    if (!mq.connected()) mq.ensureConnected();
  }

  // Publica si conectado
  if (mq.connected()) {
    String json = buildJSON(current);
    bool ok = mq.publish(MQTT_TOPIC, json);
    if (ok) {
      Serial.println(String("[MQTT] Pub -> ") + json);
    } else {
      Serial.println("[MQTT] Error al publicar.");
    }
  }
}

// vii. tarea_LED: estado WiFi/MQTT
void tarea_LED() {
  NetState st = NET_NO_INTERNET;
  if (WiFi.status() == WL_CONNECTED) {
    st = mq.connected() ? NET_MQTT_OK : NET_CONNECTING;
  }
  led.set(st);
  led.tick();
}

// ===================== Setup & Loop =====================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Hardware
  sensor.begin();
  buzzer.begin();
  led.begin();
  screen.begin();

  // RTC
  ensureRTC();

  // microSD
  if (sdcard.begin(PIN_SD_CS)) {
    Serial.println("microSD OK");
  } else {
    Serial.println("microSD NO detectada");
  }

  // WiFi & MQTT
  setupWiFi();
  mq.begin(MQTT_HOST, MQTT_PORT);

  // Marca inicial
  DateTime now = rtc.now();
  current.date = fmtDate_DDMMYYYY(now);
  current.time = fmtTime(now);
  current.sensorRaw = sensor.readRaw();
}

void loop() {
  // Mantenedores
  mq.loop();

  // Tareas periódicas sin delay
  tarea_sensor();
  tarea_actu();

  // Buzzer intermitente (cuando activo)
  buzzer.pulseTick(150, 250);  // ON 150 ms / OFF 250 ms

  tarea_pantalla();
  tarea_sd();
  tarea_mqtt();
  tarea_LED();
}
