#define USE_ARDUINO_INTERRUPTS true
//Libraries
  #include <Wire.h>
  //MAX30105.h
  #include "MAX30105.h"
  #include "heartRate.h"
  //MAX30105.h
  #include <LiquidCrystal_I2C.h> //LCD 
  #include <SoftwareSerial.h> //Wifi Module

//Constants
const int PULSE_SENSOR_PIN = A0;  // Analog PIN where the PulseSensor is connected
const int LED_PIN = 13;          // On-board LED PIN
const int THRESHOLD = 512;       // Threshold for detecting a heartbeat
const int TEMP_SENSOR = A3;      // Analog PIN where the Temperature Sensor is connected

// Variables
float tempc; //variable to store temperature in degree Celsius
float tempf; //variable to store temperature in Fahreinheit
float vout; //temporary variable to hold sensor reading

//Objects
MAX30105 particleSensor; //MAX30105.h
LiquidCrystal_I2C lcd(0x27, 16, 2); //LCD
SoftwareSerial ser(0,1); //RX, TX for Wifi Module

void MAX30102() {
if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}
 
void setup() {
  // Initialize Serial
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  MAX30102(); //MAX30102

}

void loop() {
  //LCD Init
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome");
  lcd.setCursor(0, 1);
  lcd.print("IVSMaRS");

  // Get the current Beats Per Minute (BPM)
  int currentBPM = pulseSensor.getBeatsPerMinute();

  // Reading the value from TEMP_SENSOR
  vout=analogRead(TEMP_SENSOR); 
  vout=vout*(5.00/1023); //Temperature Value
  tempc=vout*10.0; // Storing value in Degree Celsius

  // Check if a heartbeat is detected
  if (pulseSensor.sawStartOfBeat()) 
  {
    Serial.println("♥ A HeartBeat Happened!");
    Serial.print("BPM: ");
    Serial.println(currentBPM);
    Serial.print("Body Temperature: ");
    Serial.println(tempc);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(tempc);
    lcd.setCursor(0, 1);
    lcd.print("BPM: ");
    lcd.print(currentBPM);
  }
 
  // Add a small delay to reduce CPU usage
  delay(1000);
}