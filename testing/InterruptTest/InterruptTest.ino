/*
 * This Arduino Nano ESP32 code was developed by newbiely.com
 *
 * This Arduino Nano ESP32 code is made available for public use without any restriction
 *
 * For comprehensive instructions and wiring diagrams, please visit:
 * https://newbiely.com/tutorials/arduino-nano-esp32/arduino-nano-esp32-button-led
 */
#ifdef AVR
#define BUTTON_PIN 13  // The Arduino Nano ESP32 pin connected to the button
#define LED_PIN 2     // The Arduino Nano ESP32 pin connected to the LED
#else
#define BUTTON_PIN D2  // The Arduino Nano ESP32 pin connected to the button
#define LED_PIN D5     // The Arduino Nano ESP32 pin connected to the LED
#endif

int led_state = LOW;    // The current state of LED
int prev_button_state;  // The previous state of button
int button_state;       // The current state of button

void ARDUINO_ISR() {
  button_state = !button_state;
}

// void setup() {
//   Serial.begin(115200);  // Initialize the Serial to communicate with the Serial Monitor.
//   Serial.println("Begin program.");
//   pinMode(LED_PIN, OUTPUT);           // set arduino pin to output mode
//   pinMode(BUTTON_PIN, INPUT_PULLUP);  // set arduino pin to input pull-up mode
//   attachInterrupt(BUTTON_PIN, ARDUINO_ISR, CHANGE);

//   button_state = digitalRead(BUTTON_PIN);
// }

// void loop() {
//   prev_button_state = button_state;        // save the last state
//   // button_state = digitalRead(BUTTON_PIN);  // read new state

//   if (prev_button_state == HIGH && button_state == LOW) {
//     Serial.println("The button is pressed");

//     // toggle state of LED
//     led_state = !led_state;

//     // control LED according to the toggled state
//     digitalWrite(LED_PIN, led_state);
//   }
// }

// Interrupt Service Routine (ISR)
void switchPressed ()
{
  if (digitalRead (BUTTON_PIN) == HIGH)
    digitalWrite (LED_PIN, HIGH);
  else
    digitalWrite (LED_PIN, LOW);
}  // end of switchPressed

void setup ()
{
  pinMode (LED_PIN, OUTPUT);  // so we can update the LED
  digitalWrite (BUTTON_PIN, HIGH);  // internal pull-up resistor
  attachInterrupt (digitalPinToInterrupt (BUTTON_PIN), switchPressed, CHANGE);  // attach interrupt handler
}  // end of setup

void loop ()
{
  // loop doing nothing 
} 
