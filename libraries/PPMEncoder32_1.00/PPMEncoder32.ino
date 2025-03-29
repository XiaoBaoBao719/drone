#include "PPMEncoderEsp32.h"

#define OUTPUT_PIN 32

void setup() {
  ppmEncoder.begin(OUTPUT_PIN,4,true);
}

void loop() {

  // Min value
  ppmEncoder.setChannel(0, 500);
  ppmEncoder.setChannel(0, PPMEncoderEsp32::MIN);
  ppmEncoder.setChannelPercent(0, 0);
  delay(500);

  // Max value
  ppmEncoder.setChannel(0, 1500);
  ppmEncoder.setChannel(0, PPMEncoderEsp32::MAX);
  ppmEncoder.setChannelPercent(0, 100);
  delay(500);
}
