//Includo librerie
#define BLYNK_TEMPLATE_ID "ID"
#define BLYNK_TEMPLATE_NAME "NAME"
#define BLYNK_PRINT Serial
#include <ESP8266_Lib.h>
#include <ESP8266.h> //WiFi Module
#include <BlynkSimpleShieldEsp8266.h>
//Dati di accesso Wi-Fi e Blynk
char TOKEN[] = "AUTH";
char WIFI_SSID[] = "PWD";
char WIFI_PASS[] = "SSID";
//Impostazioni seriale ESP
#define BAUDRATE_ESP01 38400
#define SerialEsp Serial1
ESP8266 ESP8266_WiFi(&SerialEsp);

void setup()
{
 //Inizializzo Seriale Arduino
 Serial.begin(115200);
 delay(10);
 //Inizializzo Seriale ESP01
 Serial1.begin(BAUDRATE_ESP01);
 delay(10);
 //Attendo affinchè non si apre il Monitor Seriale
 while(!Serial);
 //Avvio Blynk
 Blynk.begin(TOKEN, ESP8266_WiFi, WIFI_SSID, WIFI_PASS);
}

void loop()
{
 //Codice loop
 Blynk.run();
} 
