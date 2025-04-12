#include <Arduino.h>

const int GPIO1 = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello, world!");
  Serial.println("This is a test of the serial communication.");
  pinMode(GPIO1, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello, world!");
  digitalWrite(GPIO1, HIGH);
  delay(1000); // Wait for a second
}

