#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiNINA.h>

#include "arduino_secrets.h"

#define triggerPin 12
#define echoPin    13

char ssid[]  = SECRET_SSID;
char pass[]  = SECRET_PASS;
int keyIndex = 0;  // your network key Index number (needed only for WEP)
int status   = WL_IDLE_STATUS;

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
IPAddress server(168,192,1,2);  // numeric IP for Bob, no DNS

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):

// MQTT client
PubSubClient client;

/*
  Measure the distance using the temperature (should there be a sensor)
*/
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



void setup() {
  Serial.begin(9600);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {  
  float d = distanceCm( 20. ); // assuming 20 °C
  Serial.print( F( "Distance: ") );
  Serial.println( d );
  delay(1000);
}