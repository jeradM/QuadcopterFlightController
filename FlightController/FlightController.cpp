#include "FlightController.h"
#include "FC_IMU.h"

extern volatile bool sensor_update_int;
RadioInput *_radio_ref;

void FlightController::init() {
  _radio_ref = &_radio;
  DDRB |= (1 << 4) | (1 << 5);
  _setup_pids();
  _imu.init();
  _radio.init();
  _motors.init();
  delay(250);
  
  while (!_initialized) {
    _radio.update(_radio_prev);
    if (_radio_prev[CH_THR] < 1130) _initialized = true;
  }
  
  _motors.init_esc();
  
  for (int i = 0; i < 5; i++) {
    _led_blue(true);
    delay(100);
    _led_blue(false);
    delay(100);
  }
  update();
}

void FlightController::calibrate_radio() {
  
}

void FlightController::update() {
  _radio.update(_radio_prev);
  _parse_aux();
  
  long in_thr = _radio_prev[CH_THR];
  long in_pit = -map(_radio_prev[CH_PITCH], _radio._channels_min[CH_PITCH], _radio._channels_max[CH_PITCH], -45, 45);
  long in_rol = map(_radio_prev[CH_ROLL], _radio._channels_min[CH_ROLL], _radio._channels_max[CH_ROLL], -45, 45);
  long in_yaw = -map(_radio_prev[CH_YAW], _radio._channels_min[CH_YAW], _radio._channels_max[CH_YAW], -150, 150);
  
  if (sensor_update_int) {
    _imu.update_sensors();
    sensor_update_int = false;
  }

  float *gyro_rate = _imu.get_gyro();
  
  if (in_thr > _radio._channels_min[CH_THR] + 100 && _motors.is_armed()) {
    float pit_err = _pids[PID_RATE_PIT].get_pid(degrees(gyro_rate[1]) - (float)in_pit);
    float rol_err = _pids[PID_RATE_ROL].get_pid(degrees(gyro_rate[0]) - (float)in_rol);
    float yaw_err = _pids[PID_RATE_YAW].get_pid(degrees(gyro_rate[2]) - (float)in_yaw);
    
    // Serial.print(pit_err);
    // Serial.print(" | ");
    // Serial.print(rol_err);
    // Serial.print(" | ");
    // Serial.print(yaw_err);
    // Serial.print(" | ");
    
    // _motors.write_pwm(MOTOR_FL, in_thr + (int16_t)(pit_err - rol_err + yaw_err));
    // _motors.write_pwm(MOTOR_FR, in_thr + (int16_t)(pit_err + rol_err - yaw_err));
    // _motors.write_pwm(MOTOR_RL, in_thr - (int16_t)(pit_err + rol_err + yaw_err));
    // _motors.write_pwm(MOTOR_RR, in_thr - (int16_t)(pit_err - rol_err - yaw_err));
    Serial.print(in_thr + (int16_t)(pit_err - rol_err + yaw_err));
    Serial.print(" | ");
    Serial.print(in_thr + (int16_t)(pit_err + rol_err - yaw_err));
    Serial.print(" | ");
    Serial.print(in_thr - (int16_t)(pit_err + rol_err + yaw_err));
    Serial.print(" | ");
    Serial.print(in_thr - (int16_t)(pit_err - rol_err - yaw_err));
    Serial.println();
  }
  else {
    _motors.write_pwm(MOTOR_FL, 1000);
    _motors.write_pwm(MOTOR_FR, 1000);
    _motors.write_pwm(MOTOR_RL, 1000);
    _motors.write_pwm(MOTOR_RR, 1000);
    if (in_thr < _radio._channels_min[CH_THR] + 30) {
      if (_radio_prev[CH_YAW] > _radio._channels_max[CH_YAW] - 50 && _motors.is_armed()) {
        _motors.disarm();
        _led_blue(false);
      }
      else if (_radio_prev[CH_YAW] < _radio._channels_min[CH_YAW] + 50 && !_motors.is_armed()) {
        _motors.arm();
        _led_blue(true);
      }
    }
  }
  
  
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
    PORTB |= (1 << 4);
  }
  else {
    PORTB &=~ (1 << 4);
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
