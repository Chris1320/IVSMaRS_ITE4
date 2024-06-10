#define USE_ARDUINO_INTERRUPTS true
//Libraries
  #include <PulseSensorPlayground.h> //Pulse Sensor
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
PulseSensorPlayground pulseSensor; //Pulse Sensor
LiquidCrystal_I2C lcd(0x27, 16, 2); //LCD
SoftwareSerial ser(0,1); //RX, TX for Wifi Module
 
void setup() {
  // Initialize Serial
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  //pinMode INPUT
  pinMode(TEMP_SENSOR,INPUT); // Configuring TEMP_SENSOR pin as input

  // PulseSensor Configuration
  pulseSensor.analogInput(PULSE_SENSOR_PIN);
  pulseSensor.blinkOnPulse(LED_PIN);
  pulseSensor.setThreshold(THRESHOLD);

  // Check if PulseSensor is initialized
  if (pulseSensor.begin()) 
  {
    Serial.println("PulseSensor object created successfully!");
  }
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