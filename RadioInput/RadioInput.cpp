#include "RadioInput.h"

void RadioInput::init() {
  PCintPort::attachInterrupt(PIN_ROLL, _read_roll, CHANGE);
  PCintPort::attachInterrupt(PIN_PITCH, _read_roll, CHANGE);
  PCintPort::attachInterrupt(PIN_THR, _read_roll, CHANGE);
  PCintPort::attachInterrupt(PIN_YAW, _read_roll, CHANGE);
  PCintPort::attachInterrupt(PIN_AUX1, _read_aux1, CHANGE);
  PCintPort::attachInterrupt(PIN_AUX2, _read_aux2, CHANGE);
  PCintPort::attachInterrupt(PIN_AUX3, _read_aux3, CHANGE);
  PCintPort::attachInterrupt(PIN_AUX4, _read_aux4, CHANGE);
}

void RadioInput::update(uint16_t channels[]) {
  for (int i = 0; i < 8; i++) {
    channels[i] = _channel_val[i];
  }
}

// Update Throttle Pulse Width
void RadioInput::_read_throttle() {
  if (_chan_mask & CH_THR) {
    if (digitalRead(PIN_THR) == HIGH) {
      _thr_start = micros();
    }
    else {
      _channels_val[CH_THR] = (uint16_t)(micros() - _thr_start);
    }
  }
}

// Update yaw Pulse Width
void RadioInput::_read_yaw() {
  if (_chan_mask & CH_YAW) {
    if (digitalRead(PIN_YAW) == HIGH) {
      _yaw_start = micros();
    }
    else {
      _channels_val[CH_YAW] = (uint16_t)(micros() - _yaw_start);
    }
  }
}

// Update Pitch Pulse Width
void RadioInput::_read_pitch() {
  if (_chan_mask & CH_PITCH) {
    if (digitalRead(PIN_PITCH) == HIGH) {
      _pitch_start = micros();
    }
    else {
      _channels_val[CH_PITCH] = (uint16_t)(micros() - _pitch_start);
    }
  }
} 

// Update Roll Pulse Width
void RadioInput::_read_roll() {
  if (_chan_mask & CH_ROLL) {
    if (digitalRead(PIN_ROLL) == HIGH) {
      _roll_start = micros();
    }
    else {
      _channels_val[CH_ROLL] = (uint16_t)(micros() - _roll_start);
    }
  }
}

void RadioInput::_read_aux1(){
  if (_chan_mask & CH_AUX1) {
    if (digitalRead(PIN_AUX1) == HIGH) {
      _aux1_start = micros();
    }
    else {
      _channels_val[CH_AUX1] = (uint16_t)(micros() - _aux1_start);
    }
  }
}

void RadioInput::_read_aux2(){
  if (_chan_mask & CH_AUX2) {
    if (digitalRead(PIN_AUX2) == HIGH) {
      _aux2_start = micros();
    }
    else {
      _channels_val[CH_AUX2] = (uint16_t)(micros() - _aux2_start);
    }
  }
}

void RadioInput::_read_aux3(){
  if (_chan_mask & CH_AUX3) {
    if (digitalRead(PIN_AUX3) == HIGH) {
      _aux3_start = micros();
    }
    else {
      _channels_val[CH_AUX3] = (uint16_t)(micros() - _aux3_start);
    }
  }
}

void RadioInput::_read_aux4(){
  if (_chan_mask & CH_AUX4) {
    if (digitalRead(PIN_AUX4) == HIGH) {
      _aux4_start = micros();
    }
    else {
      _channels_val[CH_AUX4] = (uint16_t)(micros() - _aux4_start);
    }
  }
}