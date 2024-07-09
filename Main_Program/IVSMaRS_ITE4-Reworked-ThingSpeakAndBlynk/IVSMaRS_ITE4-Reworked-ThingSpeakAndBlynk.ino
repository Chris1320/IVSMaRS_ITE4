//Libraries
  #include <Wire.h>
  //MAX30105.h
  #include "MAX30105.h"
  #include "heartRate.h"
  #include <spo2_algorithm.h>
  //MAX30105.h
  #include <LiquidCrystal_I2C.h> //LCD 
  #include <SoftwareSerial.h> //Wifi Module
  #include <ESP8266.h> //WiFi Module

// Initialize SoftwareSerial
SoftwareSerial	ConsoleOut(8, 9);
SoftwareSerial  esp8266(0, 1); // RX, TX
SoftwareSerial ser(0,1); //RX, TX for Wifi Module

//WiFi Setup
#define SSID "SSID" // Change the name of your WIFI
#define PWD "PWD" // Change the password of your WIFI

//Blynk API
const char* blynkServer = "blynk-cloud.com";
const char* authToken = "";

//Constants
const int AD8232OutputPin = A0; // AD8232 output connected to Arduino A0 pin
const int LOPlusPin = 11; // LO+ pin connected to Arduino pin 11, optional for lead-off detection
const int LOMinusPin = 10; // LO- pin connected to Arduino pin 10, optional for lead-off detection

// Variables
uint32_t irBuffer[100]; //Infrared LED buffer
uint32_t redBuffer[100];  //Red LED buffer
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //Heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


//Objects
MAX30105 particleSensor; //MAX30105.h
LiquidCrystal_I2C lcd(0x27, 20, 4); //LCD

//Thingspeak API
const String thingSpeakServer = "api.thingspeak.com";
const String thingSpeakAPIKey = "6RO8WVUUV5T3L2OP";

void MAX30102() {
if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(0x1F, 4, 3, 400, 411, 4096); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0x0A); //Turn off Green LED
  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
}

void LCD_Display(){
  lcd.init();
  lcd.backlight();
}

void LCD_Welcome(){
  lcd.clear(); // Clear the LCD screen before displaying new data
    lcd.setCursor(0, 0); // Set cursor to first line
    lcd.print("Welcome");
    lcd.setCursor(0, 1); // Move to the next line
    lcd.print("to");
    lcd.setCursor(0, 2); // Move to the third line
    lcd.print("IVSMaRS");
}

void ECG(){
  pinMode(LOPlusPin, INPUT); // Optional: for lead-off detection
  pinMode(LOMinusPin, INPUT); // Optional: for lead-off detection
}

float readMAX30102Temperature() {
  float temperature = particleSensor.readTemperature();
}

void sendCommand(String command, const int timeout, boolean debug) {
  Serial1.println(command); // Send the command to the ESP8266
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (Serial1.available()) {
      // The ESP8266 will send its response. Print it if debug is true.
      String response = Serial1.readString();
      if (debug) {
        Serial.print(response);
        lcd.clear(); // Clear the LCD screen before displaying new data
        lcd.setCursor(0, 0); // Set cursor to first line
        lcd.print(response);
        delay(1000);
      }
    }
  }
}

void connectToNetwork(){
  sendCommand("AT+CWMODE=1", 1000, true); // Set the ESP as Station
  String cmd = "AT+CWJAP=\"" + String(SSID) + "\",\"" + String(PWD) + "\"";
  sendCommand(cmd, 5000, true);
}

void initializeESP01() {
  Serial1.println("AT"); // Test AT startup
  delay(1000); // Wait for a response
  if (Serial1.available()) {
    String response = Serial1.readString();
    Serial.println("ESP01 Response: " + response); // Print response to Serial Monitor
  } else {
    Serial.println("No response from ESP01");
  }
  // Add additional AT commands as needed for your application
}

void sendReadingToThingSpeak() {
  int field1Value = heartRate;
  int field2Value = spo2;
  float field3Value = particleSensor.readTemperature();
  int field4Value = analogRead(AD8232OutputPin);

  // Construct the HTTP POST request body
  String httpRequestBody = "api_key=" + thingSpeakAPIKey + "&field1=" + String(field1Value) + "&field2=" + String(field2Value) + "&field3=" + String(field3Value) + "&field4=" + String(field4Value);
  String httpRequest = "POST /update HTTP/1.1\r\n";
  httpRequest += "Host: api.thingspeak.com\r\n";
  httpRequest += "Connection: close\r\n";
  httpRequest += "Content-Type: application/x-www-form-urlencoded\r\n";
  httpRequest += "Content-Length: " + String(httpRequestBody.length()) + "\r\n\r\n";
  httpRequest += httpRequestBody;

  // Send the HTTP POST request
  sendCommand("AT+CIPMUX=1", 1000, false); // Enable multiple connections
  sendCommand("AT+CIPSTART=0,\"TCP\",\"api.thingspeak.com\",80", 5000, false); // Start a TCP connection
  sendCommand("AT+CIPSEND=0," + String(httpRequest.length()), 5000, false); // Prepare to send data
  Serial1.println(httpRequest); // Send the HTTP request
  delay(5000); // Wait for the data to be sent
  sendCommand("AT+CIPCLOSE=0", 1000, false); // Close the TCP connection
}

void updateBlynk(int vPin, float value) {
    String cmd = "AT+CIPSTART=\"TCP\",\"" + String(blynkServer) + "\",80";
    esp8266.println(cmd);
    if(esp8266.find("OK")) {
        String httpRequest = "GET /" + String(authToken) + "/update/V" + String(vPin) + "?value=" + String(value) + " HTTP/1.1\r\nHost: " + String(blynkServer) + "\r\nConnection: close\r\n\r\n";
        cmd = "AT+CIPSEND=" + String(httpRequest.length());
        esp8266.println(cmd);
        if(esp8266.find(">")) {
            esp8266.println(httpRequest);
        }
        delay(1000); // Wait for the request to complete
        esp8266.println("AT+CIPCLOSE"); // Close the TCP connection
    }
}
 
void setup() {
  //Turn on LCD
  LCD_Display(); //LCD
  // Initialize Serial
  Serial.begin(115200);
  esp8266.begin(9600); // Start the serial communication with the Serial1
  ConsoleOut.begin(9600);
  Serial1.begin(9600); // Start the serial communication with the Serial1
  // Initialize ESP01
  initializeESP01();

  connectToNetwork(); //Connect to Network

  MAX30102(); //MAX30102

  LCD_Welcome(); //Welcome Message

  ECG(); //ECG

}

void loop() {
  // Ensure there are enough samples in the buffer for analysis
  for (int i = 0; i < 100; i++) {
    while (particleSensor.available() == false) // Wait for a measurement to become available
      particleSensor.check(); // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed(); // Read red LED
    irBuffer[i] = particleSensor.getIR(); // Read IR LED
    particleSensor.nextSample(); // Move to next sample
  }

  // Calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  float temperature = particleSensor.readTemperature(); // Read temperature from MAX30102

  // Print the results if they are valid
  if (validHeartRate && validSPO2) {
    Serial.print("Heart Rate: ");
    Serial.print(heartRate);
    Serial.print(" bpm, SpO2: ");
    Serial.print(spo2);
    Serial.println(" %");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    //LCD Output
    lcd.clear(); // Clear the LCD screen before displaying new data
    lcd.setCursor(0, 0); // Set cursor to first line
    lcd.print("Heart Rate: ");
    lcd.print(heartRate);
    lcd.print(" bpm");
    lcd.setCursor(0, 1); // Move to the next line
    lcd.print("SpO2: ");
    lcd.print(spo2);
    lcd.print(" %");
    lcd.setCursor(0, 2); // Move to the third line
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");
    sendReadingToThingSpeak(); //Send collected data to ThingSpeak API
  } else {
    lcd.clear();
    LCD_Welcome(); //Welcome Message
  }

  //ECG
  if (digitalRead(LOPlusPin) == HIGH || digitalRead(LOMinusPin) == HIGH) {
    Serial.println("Leads off detected");
  } else {
    int heartRateSignal = analogRead(AD8232OutputPin);
    Serial.print("Heart Rate Signal: ");
    Serial.println(heartRateSignal);

    
    //LCD Output
    lcd.clear();
    lcd.setCursor(0, 3); // Set cursor to fourth line 
    lcd.print("ECG: ");
    lcd.print(heartRateSignal);
    sendReadingToThingSpeak(); //Send collected data to ThingSpeak API
  }

  // Update Blynk
  int heartRateSignal = analogRead(AD8232OutputPin);
  updateBlynk(1, heartRate); // V1 for heart rate
  updateBlynk(2, spo2); // V2 for SPO2
  updateBlynk(3, temperature); // V3 for temperature
  updateBlynk(4, heartRateSignal); // V4 for ECG
  
  // Add a small delay to reduce CPU usage
  delay(1000);
}