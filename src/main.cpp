#include <Arduino.h>



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, world!");
  Serial.println("This is a test of the serial communication.");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello, world!");
  delay(1000); // Wait for a second
}

