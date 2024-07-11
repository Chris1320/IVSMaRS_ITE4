#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <spo2_algorithm.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>

// WiFi credentials
const char* ssid = "";
const char* password = "";

// Firestore REST API endpoint and credentials
const char* host = "firestore.googleapis.com";
const char* project_id = ""; // Replace with your Firestore project ID
const char* collection_name = "Saxophones";
const char* api_key = ""; // Replace with your Firestore API key

// MAX30105 sensor object
MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Variables for sensor data
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// Function to send AT commands to ESP-01
void sendATCommand(char* command, char* expected_response, unsigned long timeout) {
  Serial1.println(command);
  unsigned long start_time = millis();
  while (millis() - start_time < timeout) {
    if (Serial1.find(expected_response)) {
      break;
    }
  }
}

// Connect to WiFi
void connectToWiFi() {
  sendATCommand("AT", "OK", 1000);
  sendATCommand("AT+CWMODE=1", "OK", 1000);
  String cmd = "AT+CWJAP=\"" + String(ssid) + "\",\"" + String(password) + "\"";
  sendATCommand(cmd.c_str(), "OK", 5000);
}

// Send data to Firestore
void sendToFirestore(int heartRate, int spo2, float temperature) {
  if (WiFi.status() == WL_CONNECTED) {
    // HTTP POST request to Firestore
    String url = "https://" + String(host) + "/v1/projects/" + String(project_id) + "/databases/(default)/documents/" + String(collection_name) + "?key=" + String(api_key);
    String jsonData = "{\"fields\": {\"BMPINT\": {\"integerValue\": \"" + String(heartRate) + "\"}, \"SPOINT\": {\"integerValue\": \"" + String(spo2) + "\"}, \"TEMPINT\": {\"doubleValue\": \"" + String(temperature) + "\"}}}";

    String cmd = "AT+CIPSTART=\"TCP\",\"" + String(host) + "\",80";
    sendATCommand(cmd.c_str(), "OK", 5000);

    cmd = "POST " + url + " HTTP/1.1\r\nHost: " + String(host) + "\r\nContent-Type: application/json\r\nContent-Length: " + String(jsonData.length()) + "\r\n\r\n" + jsonData;
    Serial1.println(cmd);
    sendATCommand("", "OK", 10000);
  } else {
    Serial.println("Error in WiFi connection");
  }
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  Serial1.begin(38400); // For ESP-01 communication
  lcd.init();
  lcd.backlight();

  // Connect to WiFi
  connectToWiFi();

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }
  particleSensor.setup(0x1F, 4, 3, 400, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0x0A);
  particleSensor.enableDIETEMPRDY();
}

void loop() {
  // Read sensor data
  for (int i = 0; i < 100; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  float temperature = particleSensor.readTemperature() + 5;

  if (validHeartRate && validSPO2) {
    Serial.print("Heart Rate: ");
    Serial.print(heartRate);
    Serial.print(" bpm, SpO2: ");
    Serial.print(spo2);
    Serial.print(" %, Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    // Display data on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Heart Rate: ");
    lcd.print(heartRate);
    lcd.setCursor(0, 1);
    lcd.print("SpO2: ");
    lcd.print(spo2);
    lcd.setCursor(0, 2);
    lcd.print("Temp: ");
    lcd.print(temperature);

    // Send data to Firestore
    sendToFirestore(heartRate, spo2, temperature);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Reading Vitals...");
  }

  delay(10000); // Delay before next reading
}
