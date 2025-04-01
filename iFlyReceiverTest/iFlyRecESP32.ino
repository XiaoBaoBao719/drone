// ESP32-S3 Arduino Nano
// FS-iA6B receiver
// PPM signal from iFlySky FS-i6 controller

volatile uint64_t timer = 0;
volatile int flag = 0;
volatile ch_numr = 0;
volatile int ch[12];

hw_timer_t* timer1 = NULL;

