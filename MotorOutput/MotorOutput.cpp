#include "MotorOutput.h"

void MotorOutput::init() {
  // Setup OCR pins as output
  DDRB |= (1 << 1) | (1 << 2) | (1 << 3);
  DDRD |= (1 << 3);
  
  // Reset Timer Control Registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  
  // Setup Timer1
  // - Clear OCR on match 
  // - FastPWM w/ ICR1 TOP (255)
  // - 256 Prescaler ~244 Hz (due to Timer2 Limits)
  // - 16us tick -- PWM: 62.5 = 1000us - 125 = 2000us
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM12)  | (1 << WGM13)  | (1 << CS12);
  ICR1   = 255;
  OCR1A  = 0xFF; 
  OCR1B  = 0xFF;
  
  // Setup Timer2
  // - Clear OCR on match
  // - Fast PWM w/ default TOP (255)
  // 256 prescaler ~244 Hz
  // - 16us tick -- PWM: 62.5 = 1000us - 125 = 2000us
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS21) | (1 << CS22);
  OCR2A  = 0xFF;
  OCR2B  = 0xFF;
  
}

// Initialize ESCs at startup
void MotorOutput::init_esc() {
  if (!_esc_init) {
    for (int i = 0; i < _num_motors; i++) {
      write_pwm(i, 1000);
    }
    delay(5000);
    _esc_init = true;
  }
}

// Set PWM pulse width for an output channel
// -----
// ARGS
// -----
// channel:  the motor channel to write (0-3 from FL -> RL CW)
// in_pusle: the pulse width as read from RC input (1us units)
void MotorOutput::write_pwm(int channel, uint16_t in_pulse) {
  if (_armed) {
    uint8_t pwm = _constrain_pwm(in_pulse);
  
    switch(channel) {
      case 0: OCR1A = pwm; break;
      case 1: OCR1B = pwm; break;
      case 2: OCR2A = pwm; break;
      case 3: OCR2B = pwm; break;
    }
  }
}

// Arm motors
void MotorOutput::arm() {
  _armed = true;
}

// Disarm motors - write min pwm signal to ensure motors off
void MotorOutput::disarm() {
  for (int i = 0; i < _num_motors; i++) {
    write_pwm(i, 1000);
  }
  _armed = false;
}

// Change from 1us units to 16us units and check bounds
uint8_t MotorOutput::_constrain_pwm(uint16_t pwm) {
  uint8_t pwm_shift = (pwm >> 4); // 1us RC signal to 16us timer
  if (pwm_shift < _min_pulse) return _min_pulse;
  if (pwm_shift > _max_pulse) return _max_pulse;
  return pwm_shift;
}



