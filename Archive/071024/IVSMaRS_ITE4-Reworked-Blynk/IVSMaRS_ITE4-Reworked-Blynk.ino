//Libraries
  #include <Wire.h>
  //MAX30105.h
  #include "MAX30105.h"
  #include "heartRate.h"
  #include <spo2_algorithm.h>
  //MAX30105.h
  #include <LiquidCrystal_I2C.h> //LCD 
  #include <SoftwareSerial.h> //Wifi Module
  #include <ESP8266_Lib.h> //Blynk WiFi
  #include <ESP8266.h> //WiFi Module

// Initialize SoftwareSerial
SoftwareSerial	ConsoleOut(8, 9);
SoftwareSerial  esp8266(0, 1); // RX, TX
SoftwareSerial ser(0,1); //RX, TX for Wifi Module

//WiFi Setup
#define SSID "SSID" // Change the name of your WIFI
#define PWD "PWD" // Change the password of your WIFI

//Blynk API
#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""
#define BLYNK_PRINT Serial //Debugging
#define BAUDRATE_ESP01 38400
#define SerialEsp Serial1
ESP8266 ESP8266_WiFi(&SerialEsp);

#include <BlynkSimpleShieldEsp8266.h> //Blynk
BlynkTimer timer; //Blynk Timer

//Constants
const int AD8232OutputPin = A0; // AD8232 output connected to Arduino A0 pin
const int LOPlusPin = 11; // LO+ pin connected to Arduino pin 11, optional for lead-off detection
const int LOMinusPin = 10; // LO- pin connected to Arduino pin 10, optional for lead-off detection
#define AD8232_POWER_PIN 12

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
  float temperature = particleSensor.readTemperature() + 5;
}

//void sendCommand(String command, const int timeout, boolean debug) {
 // Serial1.println(command); // Send the command to the ESP8266
  //long int time = millis();
  //while ((time + timeout) > millis()) {
   // while (Serial1.available()) {
      // The ESP8266 will send its response. Print it if debug is true.
     // String response = Serial1.readString();
     // if (debug) {
       // Serial.print(response);
       // lcd.clear(); // Clear the LCD screen before displaying new data
       // lcd.setCursor(0, 0); // Set cursor to first line
        //lcd.print(response);
       // delay(1000);
      //}
   // }
  //}
//}

//void connectToNetwork(){
 // sendCommand("AT+CWMODE=1", 1000, true); // Set the ESP as Station
  //String cmd = "AT+CWJAP=\"" + String(SSID) + "\",\"" + String(PWD) + "\"";
  //sendCommand(cmd, 5000, true);
  //sendCommand("AT+CIPMUX=1", 1000, true); // Enable multiple connections
//}

//void initializeESP01() {
  //Serial1.println("AT"); // Test AT startup
  //delay(1000); // Wait for a response
  //if (Serial1.available()) {
   // String response = Serial1.readString();
    //Serial.println("ESP01 Response: " + response); // Print response to Serial Monitor
  //} else {
    //Serial.println("No response from ESP01");
 // }
  // Add additional AT commands as needed for your application
//}

void myTimer() {
  float temperature = particleSensor.readTemperature() + 5; // Read temperature from MAX30102
  int heartRateSignal = analogRead(AD8232OutputPin); // Read ECG signal from AD8232

  Blynk.virtualWrite(V1, heartRate); // Transmit heartRate to virtual pin V1
  Blynk.virtualWrite(V2, spo2); // Transmit spo2 to virtual pin V2
  Blynk.virtualWrite(V3, temperature); // Transmit temperature to virtual pin V3
  Blynk.virtualWrite(V4, heartRateSignal); // Transmit heartRateSignal to virtual pin V4
}

void analyzeMax(){
  float temperature = particleSensor.readTemperature() + 5; // Read temperature from MAX30102
  lcd.clear(); // Clear the LCD screen

  // Check BPM
  lcd.setCursor(0, 0); // Set cursor to first line
  lcd.print("Reading Vitals.....");
  delay(2500);
  lcd.clear(); // Clear the LCD screen
  if (heartRate >= 60 && heartRate <= 100) {
    lcd.print("BPM Normal");
  } else {
    lcd.print("BPM Abnormal");
  }

  // Check SpO2
  lcd.setCursor(0, 1); // Move to the second line
  if (spo2 >= 95 && spo2 <= 100) {
    lcd.print("SpO2 Normal");
  } else {
    lcd.print("SpO2 Low");
  }

  // Check Temperature
  lcd.setCursor(0, 2);
  if (temperature >= 36.5 && temperature <= 37.5) {
    lcd.print("Temperature Normal");
  } else {
    lcd.print("Temperature Abnormal");
  }
  delay(2500);
}

 BLYNK_WRITE(V5) { // Function to be called when something is sent to virtual pin V5
  int pinValue = param.asInt(); // Get value as integer
  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set cursor to first line
  if (pinValue == 0) {
    // Assuming 1 is the command to shut down the AD8232
    digitalWrite(AD8232_POWER_PIN, LOW); // Turn off AD8232
    Serial.println("AD8232 Shut Down");
    lcd.print("AD8232 Shut Down");

  } else {
    // Optionally, handle other commands, like turning the AD8232 back on
    digitalWrite(AD8232_POWER_PIN, HIGH); // Turn on AD8232
    Serial.println("AD8232 Powered On");
    lcd.print("AD8232 Powered On");
  }
}

void setup() {
  //Turn on LCD
  LCD_Display(); //LCD
  // Initialize Serial
  Serial.begin(115200);
  esp8266.begin(9600); // Start the serial communication with the Serial1
  ConsoleOut.begin(9600);
  Serial1.begin(BAUDRATE_ESP01); // Start the serial communication with the Serial1
  // Initialize ESP01
  //initializeESP01();

  //connectToNetwork(); //Connect to Network

  MAX30102(); //MAX30102

  LCD_Welcome(); //Welcome Message

  pinMode(AD8232_POWER_PIN, OUTPUT); // Set the AD8232 control pin as output
  digitalWrite(AD8232_POWER_PIN, HIGH); // Optionally, start with the AD8232 powered on

  ECG(); //ECG

  while(!Serial);
  Blynk.begin(BLYNK_AUTH_TOKEN, ESP8266_WiFi, SSID, PWD);
  timer.setInterval(1000L, myTimer); 

}

void loop() {
  //Blynk
  Blynk.run();
  timer.run(); //Blynk Timer

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
  float temperature = particleSensor.readTemperature() + 5; // Read temperature from MAX30102

  // Print the results if they are valid
  if (validHeartRate && validSPO2) {
    analyzeMax(); //Analyze MAX30102
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
    lcd.print("ECG: ");
    lcd.print(heartRateSignal);
  }

}