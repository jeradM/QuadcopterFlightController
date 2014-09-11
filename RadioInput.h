#ifndef __RADIO_INPUT_H__
#define __RADIO_INPUT_H__

#include "Arduino.h"

#define PIN_ROLL  8
#define PIN_PITCH 7
#define PIN_THR   6
#define PIN_YAW   5
#define PIN_AUX1  14
#define PIN_AUX2  15
#define PIN_AUX3  16
#define PIN_AUX4  17

#define CH_ROLL   0
#define CH_PITCH  1
#define CH_THR    2
#define CH_YAW    3
#define CH_AUX1   4
#define CH_AUX2   5
#define CH_AUX3   6
#define CH_AUX4   7

#include "PinChangeInt/PinChangeInt.h"

class RadioInput {
public:
  RadioInput(int num_channels) {
    _chan_mask = (1 << num_channels) - 1;
  }
  
  // Init radio input
  void init();
  
  // Decode 
  
  // Update with current channel values
  void update(uint16_t channels[]);
  
  // Set/Unset active channels
  void enable_channel(int ch)   { _chan_mask |=  (1 << ch); }
  void disable_channel(int ch)  { _chan_mask &=~ (1 << ch); }
  void toggle_channel(int ch)   { _chan_mask ^=  (1 << ch); }
  
  // Set channel bitmask (0: off - 1: on)
  void enable_mask(uint8_t mask) {
    _chan_mask = 0 | mask;
  }
  
  // Set min/max pulse width for a channel
  // These should be the pulses your transmitter outputs
  void calibrate_channel(int chan, uint16_t min, uint16_t max) {
    _channels_min[chan] = min;
    _channels_max[chan] = max;
  }
  
  
private:
  // Private Variables for channel information
  uint8_t _chan_mask = 0;
  volatile uint16_t _channels_val[8];
  uint16_t _channels_min[8];
  uint16_t _channels_max[8];
  
  // Pulse width measurement variables
  uint32_t _thr_start;
  uint32_t _yaw_start;
  uint32_t _pitch_start;
  uint32_t _roll_start;
  uint32_t _aux1_start;
  uint32_t _aux2_start;
  uint32_t _aux3_start;
  uint32_t _aux4_start;
  
  // Pin Change Interrupt methods - update _channel_val array
  void _read_throttle();
  void _read_yaw();
  void _read_pitch();
  void _read_roll();
  void _read_aux1();
  void _read_aux2(); 
  void _read_aux3();
  void _read_aux4();
  
};


#endif