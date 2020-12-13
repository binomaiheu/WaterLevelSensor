/*
  WiFi - Arduino Waterlevel sensor with MQTT interface

  Bino Maiheu

  Note : 
    In PubSubClient.h, the limit on the max message size was 128 bytes. Had to increase this to send or receive messages larger 
    than this : change the value of MQTT_MAX_PACKET_SIZE in PubSubClient.h. The library allocates this much memory in its 
    internal buffer, which reduces the memory available to the sketch itself.

*/



#include <Arduino.h>
#include <PubSubClient.h>
#include <IPAddress.h>
#include <NTPClient.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#include "arduino_secrets.h"

#define powerPin   10
#define statusPin  11
#define triggerPin 12
#define echoPin    13

#define lm35Pin     A0
#define lm35Samples 10


char data[80];

int  status = 0;

// Some timers
long tWifiConnect = 0; // timestamp of the last wifi connection attempt
long tMqttConnect = 0; // timestamp of the last wifi connection attempt


// Initialize the the wifi client
WiFiClient   wifiClient;
WiFiUDP      ntpUDP;
NTPClient    timeClient(ntpUDP);

// Setup mqtt connection
void callback(char* topic, byte* payload, unsigned int length);
PubSubClient mqttClient( SECRET_MQTT_SERVER, SECRET_MQTT_PORT, callback, wifiClient );


// Updateintervals in milliseconds
long tReadoutInterval = 5000;   // update readings every 5 seconds
long tStatusInterval  = 30000;  // update status every 30 seconds
long tReadout = 0;
long tStatus  = 0;    // timestamp of the last wifi connection status print

float readLM35( void ) {

  uint16_t temperatureLast = 0;
  uint16_t temperature = 0;

  for ( uint8_t i = 0; i < lm35Samples; i++ ) {
    // Analog range: 0 .. 1.1V = 0 .. 1023 ADC value
    // For example, LM35 analog signal of 0.182 V equals to 18.2 degree celsius
    // 1024 / 1.1V = 931 ratio between analog sample and output temperature
    // This calculation is multiplied by 1000 first to avoid expensive floating
    // point instructions.
    temperature = ((uint32_t)analogRead(lm35Pin) * 1000) / 931;
 
    if (temperatureLast == temperature) {
      // Two identical temperatures read, return
      return temperature / 10.;
    }
 
    // Different temperatures, take another sample
    temperatureLast = temperature;
  }
  return temperatureLast / 10.;
}

float distanceCm( float tempCelsius ) {

    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
    unsigned long durationMicroSec = pulseIn(echoPin, HIGH);

    double speedOfSoundInCmPerMs = 0.03313 + 0.0000606 * tempCelsius; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s
    double distanceCm = durationMicroSec / 2.0 * speedOfSoundInCmPerMs;
    if (distanceCm == 0 || distanceCm > 400) {
        return -1.0 ;
    } else {
        return distanceCm;
    }
}

// print the SSID of the network you're attached to:
void printWifiStatus() 
{  
  Serial.print("SSID: ");
  Serial.print(WiFi.SSID());  

  IPAddress ip = WiFi.localIP();
  Serial.print(", IP: ");
  Serial.print(ip);

  long rssi = WiFi.RSSI();
  Serial.print(", RSSI:");
  Serial.print(rssi);
  Serial.println(" dBm");

  return;
}


void printMacAddress() 
{
  byte mac[6];

  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
  
  return;
}

void startWifi( void ) 
{
  // attempt to connect to Wifi network:
  if ( WiFi.status() != WL_CONNECTED ) {
    delay(1);
    Serial.print("Connecting to SSID: ");
    Serial.println( SECRET_WIFI_SSID );
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.begin( SECRET_WIFI_SSID, SECRET_WIFI_PASS );

  }  

  return;
}

// do nothing when message recieved
void callback(char* topic, byte* payload, unsigned int length) {
  return;
}

void setup() {
  
  // Setup pins for reading the sensor  
  Serial.begin(9600);

  pinMode(statusPin, OUTPUT);
  digitalWrite(statusPin, LOW);
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW);

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Setup WiFI    
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("Communication with WiFi module failed!"));
    // don't continue
    while (true);
  }
  printMacAddress();

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println(F("Please upgrade the firmware"));
  }

  // clear last ADC value
  analogReference(INTERNAL1V1);
  analogRead(lm35Pin);

  Serial.println(F("Starting time client..."));

  // timeClient
  timeClient.begin();
  Serial.println(F("Setup complete..."));
}

void loop() {  

  // Set Power LED indicating device is powered & looping
  digitalWrite(powerPin, HIGH);

  // Set connection LED
  if( WiFi.status() == WL_CONNECTED && mqttClient.connected() ) 
    digitalWrite(statusPin, HIGH );
  else 
    digitalWrite(statusPin, LOW);

  /* ========================================================================
   *  WiFi management   
   * 
   *  if the wifi is not connected and the last connection attempt is more than 
   *  30 seconds ago... attempt to start the wifi agaion
   *  ==================================================================== */
  if ( ( WiFi.status() != WL_CONNECTED ) && ( millis() - tWifiConnect ) > 30000 ) {
    Serial.println(F("WiFi not connected, connecting..."));
    startWifi();
    tWifiConnect = millis();
    delay(10);
  
    // ensure we get a valid NTP
    Serial.print(F("Updating time client..."));
    while ( ! timeClient.update() ) {
      Serial.print(".");
      timeClient.forceUpdate();
    }
    Serial.println(F("OK"));
    Serial.println("Time recieved : " + timeClient.getFormattedTime() );

    printWifiStatus();
  }

  /* ========================================================================
   *  MQTT Management
   *  ==================================================================== */
  if ( ( WiFi.status() == WL_CONNECTED ) &&
       ( ! mqttClient.connected() ) && 
       ( millis() - tMqttConnect ) > 30000 ) {    
    if ( mqttClient.connect( "mqtt", SECRET_MQTT_USER, SECRET_MQTT_PASS ) ) {
      Serial.println("Connected to MQTT server.");
    } else {
      Serial.print("Failed to connect with MQTT server, state ");
      Serial.println(mqttClient.state());
    }
    tMqttConnect = millis();
  }

 
  /* ========================================================================
   *  Report status
   *  ==================================================================== */
  if ( ( millis() - tStatus ) > tStatusInterval ) {
    byte ar[6];
    WiFi.macAddress(ar);
    char macStr[18];
    sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x", ar[5], ar[4], ar[3], ar[2], ar[1], ar[0]);
    IPAddress ip = WiFi.localIP();

    String payload = \
      "{\"wifiStatus\": " + String(WiFi.status()) +
      ", \"macAddress\": \"" + macStr + "\"" +
      ", \"ipAddress\": \"" + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3] + "\""
      ", \"ssid\": \"" + WiFi.SSID() + "\"" +
      ", \"rssi\": " + String(WiFi.RSSI()) +
      ", \"mqttStatus\": " + mqttClient.connected() + "}";

    Serial.print(F("STATUS: "));
    Serial.println(payload);
    
    // transmit
    if ( mqttClient.connected() ) {        
      String chan = SECRET_MQTT_CHANNEL + String("/") + SECRET_DEVUID + String("/status");
      mqttClient.publish( chan.c_str(), payload.c_str());
    }

    tStatus = millis();
  }



  /* ========================================================================
   *  Readout sensors at interval : 
   *  ==================================================================== */
  if ( WiFi.status() == WL_CONNECTED && ( millis() - tReadout ) > tReadoutInterval ) {

    // Get dateObserved --> force update...
    while ( ! timeClient.update() ) {
      timeClient.forceUpdate();
    }

    // readout the LM35 temperature
    float temp = readLM35();
    
    // readout the distance sensor
    float d = distanceCm( temp );

    
    String payload = 
      "{\"dateObserved\": \"" + timeClient.getFormattedDate() + "\"" +
      ", \"depth\": " + String(d,2) + 
      ", \"temperature\": " + String(temp,2) + "}";
    Serial.print(F("MEASUREMENT: "));
    Serial.println(payload);

    // transmit
    if ( mqttClient.connected() ) {        
      String chan = SECRET_MQTT_CHANNEL + String("/") + SECRET_DEVUID + String("/measurement");
      if ( ! mqttClient.publish( chan.c_str(), payload.c_str()) ) {
        Serial.println("Publish failed...");
      }
    }

    // reset timer
    tReadout = millis();
  }

}