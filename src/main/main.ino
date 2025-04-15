#include "Adafruit_MPU6050.h"
#include "vl53l4cx_class.h"
#include "Wire.h"
#include "Arduino.h"

#define LED_PIN LED_BUILTIN
#define BUTTON_PIN D2

volatile byte ledState = LOW;
volatile int counts = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("start interrupt test");
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), blinkLed, RISING);
}

void loop() {
  // nothing here!
  Serial.println("counts: ");
  Serial.print(counts);
}

void blinkLed() {
  counts++;
  // ledState = !ledState;
  // digitalWrite(LED_PIN, ledState);
}