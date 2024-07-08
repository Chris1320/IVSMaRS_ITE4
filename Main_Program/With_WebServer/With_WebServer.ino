//Libraries
  #include <Wire.h>
  //MAX30105.h
  #include "MAX30105.h"
  #include "heartRate.h"
  #include <spo2_algorithm.h>
  //MAX30105.h
  #include <LiquidCrystal_I2C.h> //LCD 
  #include <SoftwareSerial.h> //Wifi Module
  #include <ESP8266.h>

//WiFi Setup
#define SSID "SSID" // Change the name of your WIFI
#define PWD "PWD" // Change the password of your WIFI
SoftwareSerial	ConsoleOut(8, 9);
SoftwareSerial esp8266(0, 1); // RX, TX
SoftwareSerial ser(0,1); //RX, TX for Wifi Module

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

void connectToNetwork(){
  String cmd = "AT+CWJAP=\"" + String(SSID) + "\",\"" + String(PWD) + "\"";
  sendCommand(cmd, 5000, true);
}

void setupWebServer() {
  sendCommand("AT+CWMODE=2", 1000, true); // Set the ESP as Station
  sendCommand("AT+CIPMUX=1", 1000, true); // Enable multiple connections
  sendCommand("AT+CIPSERVER=1,80", 1000, true); // Turn on server on port 80
}
 
 void checkClientRequest() {
  if (esp8266.available()) { // Check if the ESP8266 has any data
    String request = esp8266.readStringUntil('\r'); // Read the incoming HTTP request
    Serial.println(request); // Debugging: Print the request to the serial monitor
    
    // Simple way to detect the end of the HTTP request
    if (request.length() == 1 && request[0] == '\n') {
      sendHttpResponse();
    }
  }
}

void sendHttpResponse() {
  String httpResponse;
  httpResponse = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\n";
  httpResponse += "<head><meta http-equiv='refresh' content='5'/></head>"; // Auto-refresh every 5 seconds
  httpResponse += "<body><h1>Heart Rate and SpO2 Monitor</h1>";
  httpResponse += "<p>Heart Rate: " + String(heartRate) + "</p>";
  httpResponse += "<p>SpO2: " + String(spo2) + "%</p>";
  httpResponse += "</body></html>";

  String cipSend = "AT+CIPSEND=0," + String(httpResponse.length());
  sendCommand(cipSend, 1000, true);
  sendCommand(httpResponse, 1000, true);
  sendCommand("AT+CIPCLOSE=0", 1000, true); // Close the connection
}

void sendCommand(String command, const int timeout, boolean debug) {
  esp8266.println(command); // Send the command to the ESP8266
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (esp8266.available()) {
      // The ESP8266 will send its response. Print it if debug is true.
      String response = esp8266.readString();
      if (debug) {
        Serial.print(response);
      }
    }
  }
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  esp8266.begin(9600);
  ConsoleOut.begin(9600);

  MAX30102(); //MAX30102

  LCD_Display(); //LCD
  LCD_Welcome(); //Welcome Message

  ECG(); //ECG

  connectToNetwork(); //Connect to Network
  setupWebServer(); //Setup Web Server
}

void loop() {
  //WebServer
   checkClientRequest();
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
  }
  
  // Add a small delay to reduce CPU usage
  delay(1000);
}