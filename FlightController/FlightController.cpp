#include "FlightController.h"

void FlightController::init() {
  DDRB |= (1 << 4) | (1 << 5);
  _setup_pids();
  _imu.init();
  _radio.init();
  _motors.init();
  delay(250);
  
  while (!_initialized) {
    update(_radio_prev);
    if (_radio_prev[CH_THR] < 1100) _initialized = true;
  }
  
  _motors.init_esc();
  
  for (int i = 0; i < 5; i++) {
    _led_blue(true);
    delay(100);
    _led_blue(false);
    delay(100);
  }
  _led_blue(true);
  update();
}

void FlightController::calibrate_radio() {
  
}

void FlightController::update() {
  if (_imu.sensor_update_int) {
    _imu.update_sensors();
    _imu.sensor_update_int = false;
  }
  _radio.update(_radio_prev);
  _parse_aux();
}

void FlightController::_parse_aux() {
  _auto_level = _radio_prev[CH_AUX1] > 1500 ? true : false;
}

void FlightController::_setup_pids() {
  _pids[PID_RATE_ROL](PID_RATE_ROL_P, PID_RATE_ROL_I, PID_RATE_ROL_D, PID_RATE_ROL_IL);
  _pids[PID_RATE_PIT](PID_RATE_PIT_P, PID_RATE_PIT_I, PID_RATE_PIT_D, PID_RATE_PIT_IL);
  _pids[PID_RATE_YAW](PID_RATE_YAW_P, PID_RATE_YAW_I, PID_RATE_YAW_D, PID_RATE_YAW_IL);
  _pids[PID_STAB_ROL](PID_STAB_ROL_P, PID_STAB_ROL_I, PID_STAB_ROL_D, PID_STAB_ROL_IL);
  _pids[PID_STAB_PIT](PID_STAB_PIT_P, PID_STAB_PIT_I, PID_STAB_PIT_D, PID_STAB_PIT_IL);
  _pids[PID_STAB_YAW](PID_STAB_YAW_P, PID_STAB_YAW_I, PID_STAB_YAW_D, PID_STAB_YAW_IL);
}

void FlightController::_led_blue(bool on) {
  if (on) {
    PORTB |= (1 << LED_BLUE);
  }
  else {
    PORTB &=~ (1 << LED_BLUE);
  }
}
void FlightController::_led_green(bool on) {
  if (on) {
    PORTB |= (1 << LED_GREEN);
  }
  else {
    PORTB &=~ (1 << LED_GREEN);
  }
}
