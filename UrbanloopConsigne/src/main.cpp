#include <Arduino.h>
#include <Wire.h>

// put function declarations here:
int myFunction(int, int);

#define BAUD_RATE 9600
#define RX_PIN 16
#define TX_PIN 17


void setup() {
  Serial.begin(9600);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  if (Serial2.available()) {
    Serial.write("Flush: \n");
    while(Serial2.available()) {
    Serial.print(char(Serial2.read()));
  }
  }
  Serial2.write("Hello World!");
}

void loop() {
  while(Serial2.available()) {
    Serial.print(char(Serial2.read()));
  }
}