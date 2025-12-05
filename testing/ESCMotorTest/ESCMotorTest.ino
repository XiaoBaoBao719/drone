#include "ESC.h"
#include "Wire.h"

/* ESC & MOTOR control vars */

#define MOTOR_1_PWM (5)
#define MOTOR_2_PWM (6)
#define MOTOR_3_PWM (9)
#define MOTOR_4_PWM (10)

int motorPins[] = { 5, 6, 9, 10 };

#define SPEED_MIN (1000)
#define SPEED_MAX (1500)
#define ESC_STOP (500)
#define MIN_ARM_TIME (500)

ESC m1_esc(MOTOR_1_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m2_esc(MOTOR_2_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m3_esc(MOTOR_3_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m4_esc(MOTOR_4_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);


ESC motorESCs[] = { m1_esc, m2_esc, m3_esc, m4_esc };
int oESC;

// motor input values
float motor_one_speed, motor_two_speed, motor_three_speed, motor_four_speed;
#define MIN_THROTTLE (1180)
#define MAX_THROTTLE (1600)
#define MOTOR_CONSTANT (5)  // (7)
bool stopMotors = false;

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

void setup() {
  Serial.begin(115200);
  /* Startup animation */
  pinMode(LED_BUILTIN, HIGH);

  /* Motor setup */
  Serial.println(F("Arming ESCS..."));
  delay(2000);
  m1_esc.arm();
  m1_esc.speed(SPEED_MIN);
  Serial.println("ESC 1 armed.");
  delay(1000);
  m2_esc.arm();
  m2_esc.speed(SPEED_MIN);
  Serial.println("ESC 2 armed.");
  delay(1000);
  m3_esc.arm();
  m3_esc.speed(SPEED_MIN);
  Serial.println("ESC 3 armed.");
  delay(1000);
  m4_esc.arm();
  m4_esc.speed(SPEED_MIN);
  Serial.println("ESC 4 armed.");

  delay(5000);

  Serial.println("+++Running ESC test!+++");

  int escNum = sizeof(motorESCs) / sizeof(motorESCs[0]);

  // for (int pin = 0; pin < escNum; pin++) {

  //     Serial.print("Testing Pin: ");
  //     Serial.print(pin);

  //     for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
  //         motorESCs[pin].speed(oESC);                                    // tell ESC to go to the oESC speed value
  //         delay(10);                                            // waits 10ms for the ESC to reach speed
  //     }

  //     delay(1000);

  //     for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
  //         motorESCs[pin].speed(oESC);                                    // tell ESC to go to the oESC speed value
  //         delay(10);                                            // waits 10ms for the ESC to reach speed
  //     }
  // }

  /*  MOTOR 1  */

  Serial.print("PWM M1 on Pin: ");
  Serial.println(MOTOR_1_PWM);

  for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
    m1_esc.speed(oESC);                                   // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  delay(1000);

  for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
    m1_esc.speed(oESC);                                   // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  /*  MOTOR 2  */
  Serial.print("PWM M2 on Pin: ");
  Serial.println(MOTOR_2_PWM);

  for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
    m2_esc.speed(oESC);                                   // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  delay(1000);

  for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
    m2_esc.speed(oESC);                                   // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  /*  MOTOR 3  */
  Serial.print("PWM M3 on Pin: ");
  Serial.println(MOTOR_3_PWM);

  for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
    m3_esc.speed(oESC);                                   // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  delay(1000);

  for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
    m3_esc.speed(oESC);                                   // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  delay(1000);

  /* MOTOR 4  */
  Serial.print("PWM M4 on Pin: ");
  Serial.println(MOTOR_4_PWM);

  for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1) {  // goes from 1000 microseconds to 2000 microseconds
    m4_esc.speed(oESC);                                   // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  delay(1000);

  for (oESC = SPEED_MAX; oESC >= SPEED_MIN; oESC -= 1) {  // goes from 2000 microseconds to 1000 microseconds
    m4_esc.speed(oESC);                                   // tell ESC to go to the oESC speed value
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  Serial.println("+++Finished ESC test!+++");
}

void loop() {
}