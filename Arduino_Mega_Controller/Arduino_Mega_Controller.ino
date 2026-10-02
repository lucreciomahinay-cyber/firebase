#include <Servo.h>
#include <DHT.h>
#include <ArduinoJson.h>

// --- PIN DEFINITIONS ---
#define DHTPIN 2
#define DHTTYPE DHT22

#define SERVO_DRAIN_PIN 3

#define VALVE1_PIN 4
#define VALVE2_PIN 5
#define VALVE3_PIN 6

// Ultrasonic Sensors (Trigger & Echo)
#define TRIG1 22
#define ECHO1 23
#define TRIG2 24
#define ECHO2 25
#define TRIG3 26
#define ECHO3 27

// Analog Soil Moisture Sensors (2 per zone)
#define SOIL1_A A0
#define SOIL1_B A1
#define SOIL2_A A2
#define SOIL2_B A3
#define SOIL3_A A4
#define SOIL3_B A5

// --- SENSOR THRESHOLDS ---
const float TARGET_WATER_LEVEL_P1 = 10.0; // Desired water level in Phase 1 (cm)
const float MOISTURE_THRESHOLD_P2 = 40.0; // Target moisture percentage in Phase 2 (%)
const float MOISTURE_DRY_P3 = 15.0;       // Target dry threshold in Phase 3 (%)

// Calibration bounds for Analog Soil Moisture Sensors
const int SOIL_DRY_RAW = 850;
const int SOIL_WET_RAW = 350;

// Tank Depth for Ultrasonic Calculation
const float TANK_DEPTH_CM = 30.0;

// --- HARDWARE OBJECTS ---
DHT dht(DHTPIN, DHTTYPE);
Servo drainServo;

// --- SYSTEM STATE ---
int mode = 0;       // 0 = AUTOMATIC, 1 = MANUAL
int phase = 1;      // 1, 2, or 3

int man_v1 = 0, man_v2 = 0, man_v3 = 0, man_drain = 0;

bool state_v1 = false;
bool state_v2 = false;
bool state_v3 = false;
bool state_drain = false;

// Sensor Readings
float sm1 = 0, sm2 = 0, sm3 = 0;
float wl1 = 0, wl2 = 0, wl3 = 0;
float tempVal = 0, humVal = 0;

unsigned long lastSensorRead = 0;
unsigned long lastSerialSend = 0;

void setup() {
  Serial.begin(115200);      // USB Serial Debug
  Serial2.begin(115200);     // Serial2 to ESP32 (TX2 Pin 16 / RX2 Pin 17)

  pinMode(VALVE1_PIN, OUTPUT);
  pinMode(VALVE2_PIN, OUTPUT);
  pinMode(VALVE3_PIN, OUTPUT);
  digitalWrite(VALVE1_PIN, LOW);
  digitalWrite(VALVE2_PIN, LOW);
  digitalWrite(VALVE3_PIN, LOW);

  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);

  drainServo.attach(SERVO_DRAIN_PIN);
  drainServo.write(0); // Closed

  dht.begin();
}

void loop() {
  readIncomingCommands();

  if (millis() - lastSensorRead >= 2000) {
    lastSensorRead = millis();
    readSensors();
    executeLogic();
  }

  if (millis() - lastSerialSend >= 2000) {
    lastSerialSend = millis();
    sendTelemetryToESP();
  }
}

// --- HELPER FUNCTIONS ---
float readSoilMoisturePercent(int pin) {
  int raw = analogRead(pin);
  float pct = map(raw, SOIL_DRY_RAW, SOIL_WET_RAW, 0, 100);
  return constrain(pct, 0.0, 100.0);
}

float readUltrasonicWaterLevel(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 0.0;
  float distanceCm = (duration * 0.0343) / 2.0;
  float level = TANK_DEPTH_CM - distanceCm;
  return max(level, 0.0f);
}

void readSensors() {
  sm1 = (readSoilMoisturePercent(SOIL1_A) + readSoilMoisturePercent(SOIL1_B)) / 2.0;
  sm2 = (readSoilMoisturePercent(SOIL2_A) + readSoilMoisturePercent(SOIL2_B)) / 2.0;
  sm3 = (readSoilMoisturePercent(SOIL3_A) + readSoilMoisturePercent(SOIL3_B)) / 2.0;

  wl1 = readUltrasonicWaterLevel(TRIG1, ECHO1);
  wl2 = readUltrasonicWaterLevel(TRIG2, ECHO2);
  wl3 = readUltrasonicWaterLevel(TRIG3, ECHO3);

  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) tempVal = t;
  if (!isnan(h)) humVal = h;
}

void executeLogic() {
  if (mode == 1) { // MANUAL OVERRIDE
    state_v1 = (man_v1 == 1);
    state_v2 = (man_v2 == 1);
    state_v3 = (man_v3 == 1);
    state_drain = (man_drain == 1);
  } else {        // AUTOMATIC CONTROL
    if (phase == 1) {
      // Phase 1: Maintain water level across 3 zones & open drain on overflow
      state_v1 = (wl1 < TARGET_WATER_LEVEL_P1);
      state_v2 = (wl2 < TARGET_WATER_LEVEL_P1);
      state_v3 = (wl3 < TARGET_WATER_LEVEL_P1);

      if (wl1 > (TARGET_WATER_LEVEL_P1 + 2.0) || 
          wl2 > (TARGET_WATER_LEVEL_P1 + 2.0) || 
          wl3 > (TARGET_WATER_LEVEL_P1 + 2.0)) {
        state_drain = true;
      } else {
        state_drain = false;
      }
    } 
    else if (phase == 2) {
      // Phase 2: Maintain soil moisture without standing water
      state_drain = false;
      state_v1 = (sm1 < MOISTURE_THRESHOLD_P2);
      state_v2 = (sm2 < MOISTURE_THRESHOLD_P2);
      state_v3 = (sm3 < MOISTURE_THRESHOLD_P2);
    } 
    else if (phase == 3) {
      // Phase 3: Total drain & dry out
      state_drain = true;
      state_v1 = false;
      state_v2 = false;
      state_v3 = false;
    }
  }

  // Actuate Output Hardware
  digitalWrite(VALVE1_PIN, state_v1 ? HIGH : LOW);
  digitalWrite(VALVE2_PIN, state_v2 ? HIGH : LOW);
  digitalWrite(VALVE3_PIN, state_v3 ? HIGH : LOW);
  drainServo.write(state_drain ? 90 : 0);
}

void readIncomingCommands() {
  if (Serial2.available()) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, Serial2);
    if (!error) {
      if (doc.containsKey("mode")) mode = doc["mode"];
      if (doc.containsKey("phase")) phase = doc["phase"];
      if (doc.containsKey("man_v1")) man_v1 = doc["man_v1"];
      if (doc.containsKey("man_v2")) man_v2 = doc["man_v2"];
      if (doc.containsKey("man_v3")) man_v3 = doc["man_v3"];
      if (doc.containsKey("man_drain")) man_drain = doc["man_drain"];
    }
  }
}

void sendTelemetryToESP() {
  StaticJsonDocument<384> doc;
  doc["z1_sm"] = sm1; doc["z2_sm"] = sm2; doc["z3_sm"] = sm3;
  doc["z1_wl"] = wl1; doc["z2_wl"] = wl2; doc["z3_wl"] = wl3;
  doc["temp"] = tempVal; doc["hum"] = humVal;
  doc["phase"] = phase; doc["mode"] = mode;
  doc["v1"] = state_v1 ? 1 : 0;
  doc["v2"] = state_v2 ? 1 : 0;
  doc["v3"] = state_v3 ? 1 : 0;
  doc["drain"] = state_drain ? 1 : 0;

  serializeJson(doc, Serial2);
  Serial2.println();
}
