#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>


// Wi-Fi and MQTT configuration
const char* ssid = "ajay 4g";
const char* password = "papa1979";
const char* mqtt_server = "mqtt.eclipseprojects.io";
#define AUTO_MANUAL_PIN 13  // D7 → GPIO13

WiFiClient espClient;
PubSubClient client(espClient);

// Device identity (used in topics)
const char* device_id = "1234567890123456";
char req_topic[64], res_topic[64];

#define SLAVE_ADDR 0x12
#define REGISTER_COUNT 100
uint32_t esp_register[REGISTER_COUNT + 1];
// #define AMP_CALIBRATION_FACTOR 0.00224
#define AMP_CALIBRATION_FACTOR 0.0224
#define AVERAGE_WINDOW 5
uint32_t adc_history[AVERAGE_WINDOW];
uint8_t history_index = 0;


// Setup Wi-Fi
void setupWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
}

// Setup MQTT
void setupMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  snprintf(req_topic, sizeof(req_topic), "%s/request", device_id);
  snprintf(res_topic, sizeof(res_topic), "%s/response", device_id);
}

// MQTT reconnect logic
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(device_id)) {
      Serial.println(" connected");
      client.subscribe(req_topic);
    } else {
      Serial.print(" failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// Read 32-bit register from STM32
uint32_t readRegister32(uint8_t regIndex) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(regIndex);
  Wire.endTransmission(false);
  Wire.requestFrom(SLAVE_ADDR, 4);

  uint32_t value = 0;
  for (int i = 0; i < 4 && Wire.available(); i++) {
    value |= (Wire.read() << (8 * i));
  }
  return value;
}

// Write 32-bit register to STM32
void writeRegister32(uint8_t regIndex, uint32_t value) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(regIndex);
  for (int i = 0; i < 4; i++) {
    Wire.write((value >> (8 * i)) & 0xFF);
  }
  Wire.endTransmission();
}

// Handle MQTT message
// void callback(char* topic, byte* payload, unsigned int length) {
//   StaticJsonDocument<200> doc;
//   DeserializationError err = deserializeJson(doc, payload, length);
//   if (err) {
//     Serial.println("Invalid JSON payload.");
//     return;
//   }

//   int relay = doc["relay"];
//   int state = doc["state"];

//   if (relay < 0 || relay > 7 || (state != 0 && state != 1)) {
//     Serial.println("Invalid relay command.");
//     return;
//   }

//   writeRegister32(8 + relay, state);  // Relay registers are 8–15
//   delay(100);
//   uint32_t confirmed_state = readRegister32(8 + relay);

//   StaticJsonDocument<100> resDoc;
//   resDoc["relay"] = relay;
//   resDoc["state"] = confirmed_state;

//   char buffer[100];
//   serializeJson(resDoc, buffer);
//   client.publish(res_topic, buffer);

//   Serial.printf("Relay %d set to %d\n", relay, confirmed_state);
// }

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  if (String(topic) == req_topic) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
      Serial.println("Failed to parse JSON.");
      return;
    }

    if (doc.containsKey("relay") && doc.containsKey("state")) {
      int relay = doc["relay"];
      int state = doc["state"];

    //   if (relay >= 0 && relay <= 7 && (state == 0 || state == 1)) {
    //     uint8_t regIndex = 8 + relay;

    //     writeRegister32(regIndex, state);  // Write to STM32
    //     delay(50);  // Allow time for STM32 to process

    //     uint32_t confirmed = readRegister32(regIndex);  // Confirm state
    //     saveRelayStatesToEEPROM();

    //     StaticJsonDocument<64> res;
    //     res[String(regIndex)] = confirmed;

    //     char buffer[64];
    //     serializeJson(res, buffer);
    //     client.publish(res_topic, buffer);

    //     Serial.printf("Relay %d updated. Response: %s\n", relay, buffer);
    //   }
    if (relay >= 0 && relay <= 7 && (state == 0 || state == 1)) {
    uint8_t regIndex = 8 + relay;

    writeRegister32(regIndex, state);  // Write to STM32
    delay(50);  // Let STM32 apply it

    uint32_t confirmed = readRegister32(regIndex);  // Confirmed state
    esp_register[regIndex] = confirmed;  // ✅ Store it in esp_register
    saveRelayStatesToEEPROM();          // ✅ Save all relay states to EEPROM

    StaticJsonDocument<64> res;
    res[String(regIndex)] = confirmed;

    char buffer[64];
    serializeJson(res, buffer);
    client.publish(res_topic, buffer);

    Serial.printf("Relay %d updated. Response: %s\n", relay, buffer);
  }
 else {
        Serial.println("Invalid relay/state values.");
      }
    } 
    else if (doc.containsKey("shutdown") && doc["shutdown"] == 1) {
  Serial.println("Shutdown command received — turning off all relays.");

  StaticJsonDocument<128> res;

  for (uint8_t i = 8; i <= 15; i++) {
    writeRegister32(i, 0);         // Turn off each relay
    delay(10);                     // Allow STM32 processing
    uint32_t confirmed = readRegister32(i);
    esp_register[i] = confirmed;   // Update local state
    res[String(i)] = confirmed;    // Add to response
  }

  saveRelayStatesToEEPROM();       // Save updated relay states

  char buffer[128];
  serializeJson(res, buffer);
  client.publish(res_topic, buffer);

  Serial.println("All relays shut down. Response published:");
  Serial.println(buffer);
}

     else {
      Serial.println("relay/state keys missing, ignoring.");
    }
  }
}



// Fetch registers 0–31 from STM32
void fetchFromSTM32() {
  for (int i = 0; i <= 31; i++) {
    esp_register[i] = readRegister32(i);
  }
}

// Calculate calibrated amps into registers 32–39
// void updateCalibratedAmps() {
//   for (int i = 0; i < 8; i++) {
//     float amp = esp_register[i] * AMP_CALIBRATION_FACTOR;
//     memcpy(&esp_register[32 + i], &amp, sizeof(float));
//   }
// }
void updateCalibratedAmps() {
  for (int i = 0; i < 8; i++) {
    uint32_t raw = esp_register[i];  // ADC raw

    // === Simple moving average (only for switch 8 / index 7) ===
    if (i == 7) {
      adc_history[history_index] = raw;
      history_index = (history_index + 1) % AVERAGE_WINDOW;

      uint32_t sum = 0;
      for (int j = 0; j < AVERAGE_WINDOW; j++) {
        sum += adc_history[j];
      }
      raw = sum / AVERAGE_WINDOW;
    }

    float amp = raw * AMP_CALIBRATION_FACTOR;
    memcpy(&esp_register[32 + i], &amp, sizeof(float));
  }
}


// Print all registers 0–39 in row format
void printRegisterData() {
  Serial.println("\nRegister Dump [0–39]:");
  for (int i = 0; i <= 39; i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]=");

    if (i >= 32 && i <= 39) {
      float amp;
      memcpy(&amp, &esp_register[i], sizeof(float));
      Serial.print(amp, 2);
      Serial.print("A");
    } else {
      Serial.print(esp_register[i]);
    }

    Serial.print("  ");
    if ((i + 1) % 8 == 0) Serial.println();
  }
  Serial.println("Published...\n");
}

unsigned long lastUpdate = 0;
const unsigned long interval = 3000;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  EEPROM.begin(64);  // Allocate 64 bytes (you only need 8 for switches)
  delay(100);
  for (int i = 0; i < 8; i++) {
  uint8_t val = EEPROM.read(i);
  esp_register[8 + i] = val;

  writeRegister32(8 + i, val);
  delay(50);  // Let STM32 catch up
  Serial.printf("Restored relay %d to %d\n", i, val);
}


  pinMode(AUTO_MANUAL_PIN, INPUT); 
  setupWiFi();
  setupMQTT();
}

// void loop() {
//   if (!client.connected()) reconnectMQTT();
//   client.loop();

//   if (millis() - lastUpdate > interval) {
//     lastUpdate = millis();
//     fetchFromSTM32();
//     updateCalibratedAmps();
//     printRegisterData();
//   }
// }

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  if (millis() - lastUpdate > interval) {
    lastUpdate = millis();

    fetchFromSTM32();        // Get registers 0–31
    updateCalibratedAmps();  // Update 32–39
    printRegisterData();     // Serial debug
    int modeState = digitalRead(AUTO_MANUAL_PIN);
    esp_register[91] = (modeState == HIGH) ? 1 : 0;
    Serial.print("Auto/Manual switch state (D7/GPIO13): ");
    Serial.println(modeState == HIGH ? "AUTO" : "MANUAL");
    // === New: Publish registers 0 to 23 ===
    StaticJsonDocument<512> doc;
    for (uint8_t i = 0; i <= 23; i++) {
      doc[String(i)] = esp_register[i];
    }
    doc["91"] = esp_register[91];  // ✅ Include auto/manual mode in response


    char buffer[512];
    serializeJson(doc, buffer);
    client.publish(res_topic, buffer);
    Serial.println("Published periodic update:");
    Serial.println(buffer);
  }
}

void saveRelayStatesToEEPROM() {
  for (int i = 0; i < 8; i++) {
    EEPROM.write(i, (uint8_t)(esp_register[8 + i] & 0xFF));
  }
  EEPROM.commit();
  Serial.println("Saved relay states to EEPROM.");
}


