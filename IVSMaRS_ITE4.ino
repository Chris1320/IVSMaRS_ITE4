#define USE_ARDUINO_INTERRUPTS true

// Libraries
#include <DallasTemperature.h>  // Arduino Library for Dallas Temperature ICs
#include <LiquidCrystal_I2C.h>  // Arduino Library for I2C LCD
#include <OneWire.h>  // used to access 1-wire temperature sensors, memory and other chips.
#include <PulseSensorPlayground.h>  // Arduino Library for PulseSensor

// Constants
const int PULSE_SENSOR_PIN = A0;  // The PulseSensor
const int LED_PIN = 13;           // On-board LED PIN
const int THRESHOLD = 560;        // Threshold for detecting a heartbeat
const int TEMP_SENSOR = A3;       // The Temperature Sensor

// Variables
float tempc;  // variable to store temperature in degree Celsius
float tempf;  // variable to store temperature in Fahreinheit

// Objects
PulseSensorPlayground pulseSensor;
OneWire temp_onewire(TEMP_SENSOR);             // Setup a oneWire instance.
DallasTemperature temp_sensor(&temp_onewire);  // Pass the oneWire reference.
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  // pinMode INPUT
  pinMode(TEMP_SENSOR, INPUT);  // Configuring TEMP_SENSOR pin as input

  // PulseSensor Configuration
  pulseSensor.analogInput(PULSE_SENSOR_PIN);
  pulseSensor.blinkOnPulse(LED_PIN);
  pulseSensor.setThreshold(THRESHOLD);

  // Check if PulseSensor is initialized
  if (pulseSensor.begin()) {
    Serial.println("PulseSensor object created successfully!");
  }

  // Start temperature sensor
  temp_sensor.begin();
  Serial.println("Temperature Sensor Started");
}

void loop() {
  // LCD Init
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome");
  lcd.setCursor(0, 1);
  lcd.print("IVSMaRS");

  // Get the current Beats Per Minute (BPM)
  int currentBPM = pulseSensor.getBeatsPerMinute();

  // Reading the value from TEMP_SENSOR
  temp_sensor.requestTemperatures();
  tempc = temp_sensor.getTempCByIndex(0);  // read temperature in Celsius
                                           // NOTE: 0 is the first device

  // Check if a heartbeat is detected
  if (pulseSensor.sawStartOfBeat()) {
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
