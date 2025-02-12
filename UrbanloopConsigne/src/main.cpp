#include <Arduino.h>
#include <Wire.h>

#define BAUD_RATE 9600
#define RX_PIN 16
#define TX_PIN 17


void setup() {
  Serial.begin(9600);
  
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.write("Hello World!");
}

void loop() {
  Serial2.print('a');
  if(Serial2.available()) {
    int c = Serial2.read();
      Serial.print((char)c);
  }

}