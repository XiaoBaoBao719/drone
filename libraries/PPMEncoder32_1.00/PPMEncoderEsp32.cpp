#include "PPMEncoderEsp32.h"
#include "esp_timer.h"

PPMEncoderEsp32 ppmEncoder;

esp_timer_handle_t oneshot_timer;

void IRAM_ATTR oneshot_timer_callback( void* arg );

void PPMEncoderEsp32::begin(uint8_t pin) {
  begin(pin, PPM_DEFAULT_CHANNELS, false);
}

void PPMEncoderEsp32::begin(uint8_t pin, uint8_t ch) {
  begin(pin, ch, false);
}

void PPMEncoderEsp32::begin(uint8_t pin, uint8_t ch, boolean inverted) {
  cli();

  // Store on/off-State in variable to avoid another if in timing-critical interrupt
  onState = (inverted) ? HIGH : LOW;
  offState = (inverted) ? LOW : HIGH;
  
  pinMode(pin, OUTPUT);
  digitalWrite(pin, offState);

  enabled = true;
  state = true;
  elapsedUs = 0;
  currentChannel = 0;

  numChannels = ch;
  outputPin = pin;

  for (uint8_t ch = 0; ch < numChannels; ch++) {
    setChannelPercent(ch, 0);
  }

  const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &oneshot_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .arg = (void*) nullptr,
            .name = "one-shot"
            };
  ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));

  /* Start the timers */
  ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 500000));

  sei();
}

void PPMEncoderEsp32::setChannel(uint8_t channel, uint16_t value) {
  channels[channel] = constrain(value, PPMEncoderEsp32::MIN, PPMEncoderEsp32::MAX);
}

void PPMEncoderEsp32::setChannelPercent(uint8_t channel, uint8_t percent) {
  percent = constrain(percent, 0, 100);
  setChannel(channel, map(percent, 0, 100, PPMEncoderEsp32::MIN, PPMEncoderEsp32::MAX));
}

void PPMEncoderEsp32::enable() {
 enabled = true;
}

void PPMEncoderEsp32::disable() {
 enabled = false;
 state = false;
 elapsedUs = 0;
 currentChannel = 0;
 
 digitalWrite(outputPin, offState);
}

void PPMEncoderEsp32::interrupt() 
  {
  if (!enabled) {
    return;
  }

  if (state) {
    digitalWrite(outputPin, onState);
    ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, PPM_PULSE_LENGTH_uS));
    elapsedUs += PPM_PULSE_LENGTH_uS;
    } 
  else 
    {
    digitalWrite(outputPin, offState);
    if (currentChannel >= numChannels) 
      {
      currentChannel = 0;
      elapsedUs = elapsedUs + PPM_PULSE_LENGTH_uS;
      ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, PPM_FRAME_LENGTH_uS-elapsedUs));
      elapsedUs = 0;
      } 
    else 
      {
      ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, channels[currentChannel]));
      elapsedUs += channels[currentChannel];
      currentChannel++;
      }
    }
  state = !state;
  }

void IRAM_ATTR oneshot_timer_callback( void* arg )
  {
  ppmEncoder.interrupt();
  }
