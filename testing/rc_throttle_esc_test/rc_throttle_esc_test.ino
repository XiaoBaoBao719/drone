#include "ESC.h"
#include "Wire.h"
#include "ppm.h"
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* ESC & MOTOR control vars */

#define MOTOR_1_PWM (5)
#define MOTOR_2_PWM (6)
#define MOTOR_3_PWM (9)
#define MOTOR_4_PWM (10)

int motorPins[] = { 5, 6, 9, 10 };

#define SPEED_MIN (1000)
#define SPEED_MAX (2000)
#define ESC_STOP (500)
#define MIN_ARM_TIME (500)

ESC m1_esc(MOTOR_1_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m2_esc(MOTOR_2_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m3_esc(MOTOR_3_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m4_esc(MOTOR_4_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);


ESC motorESCs[] = { m1_esc, m2_esc, m3_esc, m4_esc };
int oESC;

/* Quadcopter Motor Layout

   (M4)   (Front-X)   (M1)
      \\            //
       \\   ----   //
         |        |
(Left-Y) |  (Z)   |   (Right)
         |        |
       //   ----   \\
      //            \\
   (M3)   (Back)   (M2)

   M1 - CCW
   M2 - CW
   M3 - CCW
   M4 - CW
*/

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  /* Startup animation */
  pinMode(LED_BUILTIN, HIGH);

  // PPM on pin for iFlyReceiver
  ppm.begin(interruptPin, false);

  /* Motor setup */
  // Serial.println("Calibrating ESCS...");
  // m1_esc.calib();
  // m1_esc.stop();

  // m2_esc.calib();
  // m2_esc.stop();

  // m3_esc.calib();
  // m3_esc.stop();

  // m4_esc.calib();
  // m4_esc.stop();

  Serial.println(F("Arming ESCS..."));
  delay(2000);
  m1_esc.arm();
  // m1_esc.speed(SPEED_MIN);
  Serial.println("ESC 1 armed.");
  delay(1000);
  m2_esc.arm();
  // m2_esc.speed(SPEED_MIN);
  Serial.println("ESC 2 armed.");
  delay(1000);
  m3_esc.arm();
  // m3_esc.speed(SPEED_MIN);
  Serial.println("ESC 3 armed.");
  delay(1000);
  m4_esc.arm();
  // m4_esc.speed(SPEED_MIN);
  Serial.println("ESC 4 armed.");

  // m2_esc.setCalibrationDelay(uint32_t calibration_delay)
  delay(5000);
}
short throttle = 0;
void loop() {
  // put your main code here, to run repeatedly:

  // limit ppm read to specific frequency
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    throttle      =   ppm.read_channel(THROTTLE);
    Serial.print("Throttle:");
    Serial.println(throttle);

    // Map input between allowable PWM band
    float esc_speed_out = map(throttle, 1000, 2000, SPEED_MIN, SPEED_MAX);
    m2_esc.speed(esc_speed_out);
    Serial.print("esc_out:");
    Serial.println(esc_speed_out);
  }
  

  
}
