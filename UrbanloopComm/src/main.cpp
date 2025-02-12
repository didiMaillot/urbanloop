#include <Arduino.h>
#include <Wire.h>

#define BAUD_RATE 9600
#define RX_PIN 16
#define TX_PIN 17

#define PIN_SPEED 12
#define PIN_ON 14


void setup() {
  Serial.begin(9600);
  
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.write("Hello World!");
  pinMode(PIN_SPEED, OUTPUT);
  pinMode(PIN_ON, OUTPUT);
}

bool on = false;
uint8_t speed = 0;

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    String str("found char ");
    str += c;
    Serial.println(str);
    if (c == '1') {
      on = true;
    } else  if (c == '0') {
      on = false;
    }
    digitalWrite(PIN_ON, on);
  }

  if(Serial2.available()) {
    speed = Serial2.read();
    String str("new speed is ");
    str += speed;
    Serial.println(str);
    analogWrite(PIN_SPEED, speed);
  }

}