#include "ppm.h"

#define ROLL     1

const long interval = 50;
unsigned long previousMillis = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  ppm.begin(2, false);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  }

  short roll = ppm.read_channel(ROLL);

  // print values

  Serial.print("Roll: ");   Serial.print(roll);   Serial.print("  ");
  Serial.println( );
}

// volatile byte state = LOW;
// volatile byte previous_state= LOW;
// volatile unsigned long microsAtLastPulse=0;
// volatile unsigned long blankTime=2100;
// volatile int channelAmount=6;
// volatile int pulseCounter=0;
// volatile unsigned long rawValues[10];

// void setup()
// {
//   Serial.begin(115200);
//   pinMode(2, INPUT_PULLUP);
//   attachInterrupt(2, pulseCatcherISR, RISING);
//  }
// unsigned long getRadioPPM(int ch_num) {
// //call this with the channel number to get the PPM value
//   unsigned long valPWM = 0;
//   valPWM = rawChannelValue(ch_num);// 1000-2000 based on PPM routine return range.
//   return valPWM;
// }

// void pulseCatcherISR() {
//   //Remember the current micros() and calculate the time since the last pulseReceived()
//   unsigned long previousMicros = microsAtLastPulse;
//   microsAtLastPulse = micros();
//   unsigned long time = microsAtLastPulse - previousMicros;
  
//   if (time > blankTime) {
//       // Blank detected: restart from channel 1 
//       pulseCounter = 0;
//       state=true;
//   } 
//   else {
//       // Store times between pulses as channel values
//       state=false;
//       if (pulseCounter < channelAmount) {
//           rawValues[pulseCounter] = time;
//           ++pulseCounter;
//       }
//   }
// }

// unsigned long rawChannelValue(int channel) {
//     // Check for channel's validity and return the latest raw channel value or 0
//     unsigned value = 0;
//     if (channel >= 1 && channel <= channelAmount) {
//         //noInterrupts();
//         value = rawValues[channel-1];
//        //interrupts();
//     }
//     return value;
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   for (int i = 1; i <= channelAmount; ++i) {
//     unsigned long rxvalue = getRadioPPM(i);
//     Serial.print("Channel ");
//     Serial.print(i);
//     Serial.print(": ");
//     Serial.println(rxvalue);
//   }
//   delay(1000); // Delay to slow down the printing
// }