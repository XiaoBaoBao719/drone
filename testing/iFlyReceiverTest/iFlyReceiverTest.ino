// #include "Arduino.h"

// #define ARDUINO_ARCH_RENESAS

// #ifdef ARDUINO_AVR_UNO
//   #include "ppm.h"
// #elif defined(ARDUINO_ARCH_RENESAS)
//   #include "PPMReader.h"
// #endif

// #define PIN_LED (13)
// #define ROLL     1
// #define PPM_INTERRUPT (3)

// /* FS-iA6B channel map
//  *  Ch 1 -  Yaw
//  *  Ch 2 -  Pitch
//  *  Ch 3 -  Throttle
//  *  Ch 4 -  Roll
//  *  Ch 5 -  L knob
//  *  Ch 6 -  R knob
//  */
// #define CH_YAW (1)
// #define CH_ROLL (4)
// #define CH_THR (3)
// #define CH_PITCH (2)
// #define CH_L_KNOB (5)
// #define CH_R_KNOB (6)

// const long interval = 50;
// unsigned long previousMillis = 0;

// const int NUM_CHANNELS = 6;
// float inputX, inputY, inputZ, input_throttle;

// // #ifdef ARDUINO_ARCH_RENESAS
//   PPMReader ppm_(PPM_INTERRUPT, NUM_CHANNELS);
//   int32_t receiverValue[NUM_CHANNELS] = { 0.0 };
//   int32_t long value = 0.0;

// void readReceiver(void) {
//   /* read latest valid values from all channels */
//   digitalWrite(PIN_LED, HIGH); // Turn LED on
//   for (int channel = 1; channel <= NUM_CHANNELS; channel++) {
//     value = ppm_.latestValidChannelValue(channel, 0);
//     receiverValue[channel - 1] = value;
//     Serial.print(String(value) + " ");
//   }
//   Serial.println();
// }
// // #endif 

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Starting RC receiver test.");
//   pinMode(PIN_LED, OUTPUT);    // Set pin 13 as output
  
//   #ifdef ARDUINO_AVR_UNO
//       ppm.begin(PPM_INTERRUPT, false);
//   #endif

//   // #ifdef ARDUINO_ARCH_RENESAS
//   //     uint32_t init_throttle = receiverValue[ROLL];
//   //     while (init_throttle >= 1100) {
//   //       readReceiver();
//   //       delay(100);
//   //     }
//   // #endif 
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   unsigned long currentMillis = millis();

//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis;
//     Serial.println(currentMillis);
//   }

// #ifdef ARDUINO_AVR_UNO
//   short roll = ppm.read_channel(ROLL);
//   // print values
//   Serial.print("Roll: ");   Serial.print(roll);   Serial.print("  ");
//   Serial.println( );
// #endif 

// // #ifdef ARDUINO_ARCH_RENESAS
//   readReceiver();               // read PPM from the RC remote

//   // if (receiverValue[CH_THR - 1] < 1180) {
//   //   return;
//   // }
//   input_throttle = 0.15 * (receiverValue[CH_THR - 1]);
//   inputX = 0.15 * (receiverValue[CH_ROLL - 1] - 1500);   // roll
//   inputY = 0.15 * (receiverValue[CH_PITCH - 1] - 1500);  // pitch
//   inputZ = 0.15 * (receiverValue[CH_YAW - 1] - 1500);    // yaw

//   // Serial.print(String(input_throttle)+"\n");
//   // Serial.print(String(inputX)+"\n");
//   // Serial.print(String(inputY)+"\n");
//   // Serial.print(String(inputZ)+"\n");
//   Serial.println("test");
//   // Serial.println("inX: " + String(inputX) + "\tinY: " + String(inputY) + "\tinZ: " + String(inputZ));
//   delay(100);
//   digitalWrite(PIN_LED, LOW);
// // #endif
// }

// // volatile byte state = LOW;
// // volatile byte previous_state= LOW;
// // volatile unsigned long microsAtLastPulse=0;
// // volatile unsigned long blankTime=2100;
// // volatile int channelAmount=6;
// // volatile int pulseCounter=0;
// // volatile unsigned long rawValues[10];

// // void setup()
// // {
// //   Serial.begin(115200);
// //   pinMode(2, INPUT_PULLUP);
// //   attachInterrupt(2, pulseCatcherISR, RISING);
// //  }
// // unsigned long getRadioPPM(int ch_num) {
// // //call this with the channel number to get the PPM value
// //   unsigned long valPWM = 0;
// //   valPWM = rawChannelValue(ch_num);// 1000-2000 based on PPM routine return range.
// //   return valPWM;
// // }

// // void pulseCatcherISR() {
// //   //Remember the current micros() and calculate the time since the last pulseReceived()
// //   unsigned long previousMicros = microsAtLastPulse;
// //   microsAtLastPulse = micros();
// //   unsigned long time = microsAtLastPulse - previousMicros;
  
// //   if (time > blankTime) {
// //       // Blank detected: restart from channel 1 
// //       pulseCounter = 0;
// //       state=true;
// //   } 
// //   else {
// //       // Store times between pulses as channel values
// //       state=false;
// //       if (pulseCounter < channelAmount) {
// //           rawValues[pulseCounter] = time;
// //           ++pulseCounter;
// //       }
// //   }
// // }

// // unsigned long rawChannelValue(int channel) {
// //     // Check for channel's validity and return the latest raw channel value or 0
// //     unsigned value = 0;
// //     if (channel >= 1 && channel <= channelAmount) {
// //         //noInterrupts();
// //         value = rawValues[channel-1];
// //        //interrupts();
// //     }
// //     return value;
// // }

// // void loop() {
// //   // put your main code here, to run repeatedly:
// //   for (int i = 1; i <= channelAmount; ++i) {
// //     unsigned long rxvalue = getRadioPPM(i);
// //     Serial.print("Channel ");
// //     Serial.print(i);
// //     Serial.print(": ");
// //     Serial.println(rxvalue);
// //   }
// //   delay(1000); // Delay to slow down the printing
// // }

/*
This example outputs values from all PPM channels to Serial
in a format compatible with Arduino IDE Serial Plotter
*/

// #include <PPMReader.h>
// #include "/home/metis/drone/libraries/PPMReader/PPMReader.h"
#include "ppm.h"

/* FS-iA6B channel map
 *  Ch 1 -  Yaw
 *  Ch 2 -  Pitch
 *  Ch 3 -  Throttle
 *  Ch 4 -  Roll
 *  Ch 5 -  L knob
 *  Ch 6 -  R knob
 */

// PPM channel layout (update for your situation)
#define THROTTLE        3
#define ROLL            1
#define PITCH           2
#define YAW             4
#define SWITCH3WAY_1    5
#define BUTTON          6
#define SWITCH3WAY_2    7     // (NOT USED) trim-pot for left/right motor mix  (face trim)
#define POT             8     // (NOT USED) trim-pot on the (front left edge trim)


// Loop interval time
const long interval = 50;
unsigned long previousMillis = 0;

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
byte interruptPin = 3;
byte channelAmount = 6;
// PPMReader ppm(interruptPin, channelAmount);

void setup() {
    Serial.begin(115200);
    // Serial.print(ppm.blankTime);
    ppm.begin(interruptPin, false);


}

void loop() {
    // Print latest valid values from all channels
    // for (byte channel = 1; channel <= channelAmount; ++channel) {
    //     unsigned value = ppm.latestValidChannelValue(channel, 0);
    //     // Serial.print(value);
    //     if(channel < channelAmount) Serial.print('\t');
    // }
    // Serial.println("blah");
    // delay(20);

    // Interval at which the PPM is updated
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Acquiring all the channels values
    short throttle      =   ppm.read_channel(THROTTLE);
    short roll          =   ppm.read_channel(ROLL);
    short pitch         =   ppm.read_channel(PITCH);
    short yaw           =   ppm.read_channel(YAW);
    short switch3way_1  =   ppm.read_channel(SWITCH3WAY_1);
    short button        =   ppm.read_channel(BUTTON);
    short switch3way_2  =   ppm.read_channel(SWITCH3WAY_2);
    short pot           =   ppm.read_channel(POT);

    // Print the values for the Arduino Serial Plotter
    Serial.print("Throttle:");        Serial.print(throttle);       Serial.print(" ");
    Serial.print("Roll:");            Serial.print(roll);           Serial.print(" ");
    Serial.print("Pitch:");           Serial.print(pitch);          Serial.print(" ");
    Serial.print("Yaw:");             Serial.print(yaw);            Serial.print(" ");
    Serial.print("Switch_3way_1:");   Serial.print(switch3way_1);   Serial.print(" ");
    Serial.print("Button:");          Serial.print(button);         Serial.print(" ");
    Serial.print("Switch_3way_2:");   Serial.print(switch3way_2);   Serial.print(" ");
    Serial.print("Pot:");             Serial.print(pot);            Serial.print(" ");
    Serial.println();
  }
}
