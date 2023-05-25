#include "PPMEncoder.h"

PPMEncoder ppmEncoder;

int PPM_CLOCK_MULTIPLIER=F_CPU/8000000;

  void PPMEncoder::begin(uint8_t pin) {
    begin(pin, PPM_DEFAULT_CHANNELS, false);
  }

  void PPMEncoder::begin(uint8_t pin, uint8_t ch) {
    begin(pin, ch, false);
  }

  void PPMEncoder::begin(uint8_t pin, uint8_t ch, boolean inverted) {
    cli();

    // Store on/off-State in variable to avoid another if in timing-critical interrupt
    onState = (inverted) ? HIGH : LOW;
    offState = (inverted) ? LOW : HIGH;
    
    pinMode(pin, OUTPUT);
    digitalWrite(pin, offState);

    state = true;
    elapsedUs = 0;
    currentChannel = 0;

    numChannels = ch;
    outputPin = pin;

    for (uint8_t ch = 0; ch < numChannels; ch++) {
      setChannelPercent(ch, 0);
    }

  #ifdef PPM_TIMER_2
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;

    OCR2A = 250;
    TCCR2B |= (1 << WGM21);
  // Set CS22, CS21 and CS20 bits for 8 prescaler
    TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20);
    TIMSK2 = (1 << OCIE2A); // enable timer compare interrupt
  #else
    TCCR1A = 0;

    OCR1A = 200;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    TIMSK1 = (1 << OCIE1A); // enable timer compare interrupt
  #endif
    sei();
  }

  void PPMEncoder::setChannel(uint8_t channel, uint16_t value) {
    channels[channel] = constrain(value, PPMEncoder::MIN, PPMEncoder::MAX);
  }

  void PPMEncoder::setChannelPercent(uint8_t channel, uint8_t percent) {
    percent = constrain(percent, 0, 100);
    setChannel(channel, map(percent, 0, 100, PPMEncoder::MIN, PPMEncoder::MAX));
  }


#ifdef PPM_TIMER_2

  void PPMEncoder::interrupt() {
      TCNT2 = 0;
     if (istate == 0 || (istate == 4 && remaining < 20)) {
       digitalWrite(outputPin, onState);    
       if(istate==4) {
         currentChannel++;   
          if (currentChannel > numChannels) {
                currentChannel = 0;              
            }   
      	}       

      if(PPM_CLOCK_MULTIPLIER==1) {
        istate=3;
      } else {
        istate=1;
      }
      OCR2A = 250;
    } else if(PPM_CLOCK_MULTIPLIER > 1 && istate<3) {
      istate=istate+1;
      OCR2A = 250;
    } else if(istate==3) {
      istate=istate+1;
      if(currentChannel>=numChannels) {
        remaining = (PPM_FRAME_LENGTH_uS - elapsedUs - PPM_PULSE_LENGTH_uS) * PPM_CLOCK_MULTIPLIER;
        elapsedUs = 0;
      } else {
        remaining = (channels[currentChannel]-PPM_PULSE_LENGTH_uS ) * PPM_CLOCK_MULTIPLIER ;
        elapsedUs = elapsedUs + channels[currentChannel] ;
      }
    } else {
        digitalWrite(outputPin, offState);
        if(remaining > 250) {          
          remaining = remaining - 250;          
          OCR2A = 250;
        } else {          
          OCR2A = remaining;
          istate = 0;    
          currentChannel++;   
          if (currentChannel > numChannels) {
                currentChannel = 0;              
            }   
        }        
    }
  }

 
  ISR(TIMER2_COMPA_vect) {
    ppmEncoder.interrupt();
  }
 #else
	 
 void PPMEncoder::interrupt() {

  TCNT1 = 0;

  if (state) {
    digitalWrite(outputPin, onState);
    OCR1A = PPM_PULSE_LENGTH_uS * PPM_CLOCK_MULTIPLIER;

  } else {
    digitalWrite(outputPin, offState);

    if (currentChannel >= numChannels) {
      currentChannel = 0;
      elapsedUs = elapsedUs + PPM_PULSE_LENGTH_uS;
      OCR1A = (PPM_FRAME_LENGTH_uS - elapsedUs) * PPM_CLOCK_MULTIPLIER;
      elapsedUs = 0;
    } else {
      OCR1A = (channels[currentChannel] - PPM_PULSE_LENGTH_uS) * PPM_CLOCK_MULTIPLIER;
      elapsedUs = elapsedUs + channels[currentChannel];

      currentChannel++;
    }
  }

  state = !state;
}

  ISR(TIMER1_COMPA_vect) {
    ppmEncoder.interrupt();
  }
 #endif
