#include <Arduino.h>
#include <PubSubClient.h>
#include <IPAddress.h>
#include <WiFiNINA.h>

#include "arduino_secrets.h"

#define triggerPin 12
#define echoPin    13

char data[80];

int  status = 0;

// Some timers
long tWifiConnect = -1; // timestamp of the last wifi connection attempt
long tMqttConnect = -1; // timestamp of the last wifi connection attempt
long tWifiStatus  = -1; // timestamp of the last wifi connection status print

// Initialize the the wifi client
WiFiClient   wifiClient;

// Setup mqtt connection
void callback(char* topic, byte* payload, unsigned int length);
PubSubClient mqttClient( SECRET_MQTT_SERVER, SECRET_MQTT_PORT, callback, wifiClient );

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

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Setup WiFI    
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  printMacAddress();


  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

}

void loop() {  

  /* ========================================================================
   *  WiFi management   
   *  ==================================================================== */

  // if the wifi is not connected and the last connection attempt is more than 
  // 30 seconds ago... attempt to start the wifi agaion
  if ( ( WiFi.status() != WL_CONNECTED ) && ( millis() - tWifiConnect ) > 30000 ) {
    startWifi();
    tWifiConnect = millis();
    delay(10);
    printWifiStatus();
  }

  // print wifi status every 60 sec
  if ( ( WiFi.status() == WL_CONNECTED ) && ( millis() - tWifiStatus ) > 60000 ) {
    printWifiStatus();
    tWifiStatus = millis();
  }



  /* ========================================================================
   *  Readout sensors
   *  ==================================================================== */
  float d = distanceCm( 20. ); // assuming 20 °C
  Serial.print( F( "Distance: ") );
  Serial.println( d );

  status = 0.;
  if ( d < 0. ) status = 1;



  /* ========================================================================
   *  MQTT Broker
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

  if ( mqttClient.connected() ) {
    long rssi = WiFi.RSSI();
    String payload = 
      "{\"depth\": " + String(d,2) + 
      ", \"rssi\": " + String(rssi) + 
      ", \"status\": " + String(status) + "}";
    payload.toCharArray(data, (payload.length() + 1));
    mqttClient.publish( SECRET_MQTT_CHANNEL, data);
  }


  // and repeat...
  delay(5000);
}