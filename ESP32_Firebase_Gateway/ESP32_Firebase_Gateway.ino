#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_NAME"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

#ifndef API_KEY
#define API_KEY "YOUR_FIREBASE_API_KEY"
#endif

#ifndef DATABASE_URL
#define DATABASE_URL "https://your-project-id-default-rtdb.firebaseio.com/"
#endif

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long lastFirebaseSync = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX2 = GPIO 16, TX2 = GPIO 17

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.signer.test_mode = true;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  // 1. Read Telemetry from Arduino Mega (Serial2) and Push to Firebase
  if (Serial2.available()) {
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, Serial2);
    if (!err) {
      if (Firebase.ready()) {
        Firebase.RTDB.setFloat(&fbdo, "/telemetry/z1_sm", doc["z1_sm"]);
        Firebase.RTDB.setFloat(&fbdo, "/telemetry/z2_sm", doc["z2_sm"]);
        Firebase.RTDB.setFloat(&fbdo, "/telemetry/z3_sm", doc["z3_sm"]);
        Firebase.RTDB.setFloat(&fbdo, "/telemetry/z1_wl", doc["z1_wl"]);
        Firebase.RTDB.setFloat(&fbdo, "/telemetry/z2_wl", doc["z2_wl"]);
        Firebase.RTDB.setFloat(&fbdo, "/telemetry/z3_wl", doc["z3_wl"]);
        Firebase.RTDB.setFloat(&fbdo, "/telemetry/temp", doc["temp"]);
        Firebase.RTDB.setFloat(&fbdo, "/telemetry/hum", doc["hum"]);
        Firebase.RTDB.setInt(&fbdo, "/status/v1", doc["v1"]);
        Firebase.RTDB.setInt(&fbdo, "/status/v2", doc["v2"]);
        Firebase.RTDB.setInt(&fbdo, "/status/v3", doc["v3"]);
        Firebase.RTDB.setInt(&fbdo, "/status/drain", doc["drain"]);
      }
    }
  }

  // 2. Fetch Control Commands from Firebase and Send to Arduino Mega
  if (Firebase.ready() && (millis() - lastFirebaseSync >= 1000)) {
    lastFirebaseSync = millis();

    StaticJsonDocument<256> cmdDoc;
    if (Firebase.RTDB.getInt(&fbdo, "/controls/mode")) cmdDoc["mode"] = fbdo.intData();
    if (Firebase.RTDB.getInt(&fbdo, "/controls/phase")) cmdDoc["phase"] = fbdo.intData();
    if (Firebase.RTDB.getInt(&fbdo, "/controls/man_v1")) cmdDoc["man_v1"] = fbdo.intData();
    if (Firebase.RTDB.getInt(&fbdo, "/controls/man_v2")) cmdDoc["man_v2"] = fbdo.intData();
    if (Firebase.RTDB.getInt(&fbdo, "/controls/man_v3")) cmdDoc["man_v3"] = fbdo.intData();
    if (Firebase.RTDB.getInt(&fbdo, "/controls/man_drain")) cmdDoc["man_drain"] = fbdo.intData();

    serializeJson(cmdDoc, Serial2);
    Serial2.println();
  }
}
