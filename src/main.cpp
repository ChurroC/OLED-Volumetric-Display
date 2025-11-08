#include <Arduino.h>

#define LED 2

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Set the LED HIGH
  digitalWrite(LED_BUILTIN, HIGH);
 
  // Wait for a second
  delay(1000);
 
  // Set the LED LOW
  digitalWrite(LED_BUILTIN, LOW);
 
   // Wait for a second
  delay(1000);
}