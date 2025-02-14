#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <time.h>

#define BAUD_RATE 9600
#define RX_PIN 16
#define TX_PIN 17

int vitesse() {
  return rand() % 255 + 1;
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  srand(time(NULL));
  Serial2.write("Hello World!");
}

void loop() {
  Serial2.write(vitesse());
  delay(5000);
}

