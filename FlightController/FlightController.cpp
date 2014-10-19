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
  //while (!sensor_update_int);
  //_imu.update_sensors();
  sensor_update_int = false;
  
  _radio.update(_radio_prev);
  _parse_aux();
  
  long in_thr = map(_radio_prev[CH_THR], _radio._channels_min[CH_THR], _radio._channels_max[CH_THR], 1000, 1850);
  long in_pit, in_rol, in_yaw;

  if (_auto_level) {
    _led_green(true);
    in_pit = -map(_radio_prev[CH_PITCH], _radio._channels_min[CH_PITCH], _radio._channels_max[CH_PITCH], -45, 45);
    in_rol = map(_radio_prev[CH_ROLL], _radio._channels_min[CH_ROLL], _radio._channels_max[CH_ROLL], -45, 45);
    in_yaw = -map(_radio_prev[CH_YAW], _radio._channels_min[CH_YAW], _radio._channels_max[CH_YAW], -150, 150);
  }
  else {
    _led_green(false);
    in_pit = -map(_radio_prev[CH_PITCH], _radio._channels_min[CH_PITCH], _radio._channels_max[CH_PITCH], -90, 90);
    in_rol = map(_radio_prev[CH_ROLL], _radio._channels_min[CH_ROLL], _radio._channels_max[CH_ROLL], -90, 90);
    in_yaw = -map(_radio_prev[CH_YAW], _radio._channels_min[CH_YAW], _radio._channels_max[CH_YAW], -150, 150);
  }

  float *gyro_rate = _imu.get_gyro();
  
  if (in_thr > 1100 && _motors.is_armed()) {
    float pit_err, rol_err, yaw_err;
    if (_auto_level) {
      float eulers[3];
      _imu._quaternion.to_euler(eulers);
      float pit_stab_err = _pids[PID_STAB_PIT].get_pid((float)in_pit - degrees(eulers[1]));
      float rol_stab_err = _pids[PID_STAB_ROL].get_pid((float)in_rol - degrees(eulers[0]));
      float yaw_stab_err = _pids[PID_STAB_YAW].get_pid((float)in_yaw - degrees(eulers[2]));
      
      // Serial.print(degrees(eulers[1]));
      // Serial.print(" | ");
      // Serial.print(degrees(eulers[0]));
      // Serial.print(" | ");
      // Serial.print(degrees(eulers[2]));
      // Serial.print(" | ");
      // Serial.print(in_pit);
      // Serial.print(" | ");
      // Serial.print(in_rol);
      // Serial.print(" | ");
      // Serial.print(in_yaw);
      // Serial.print(" | ");
      
      pit_err = _pids[PID_RATE_PIT].get_pid(degrees(gyro_rate[1]) - pit_stab_err);
      rol_err = _pids[PID_RATE_ROL].get_pid(degrees(gyro_rate[0]) - rol_stab_err);
      yaw_err = _pids[PID_RATE_YAW].get_pid(degrees(gyro_rate[2]) - yaw_stab_err);
    }
    else {
      pit_err = _pids[PID_RATE_PIT].get_pid(degrees(gyro_rate[1]) - (float)in_pit);
      rol_err = _pids[PID_RATE_ROL].get_pid(degrees(gyro_rate[0]) - (float)in_rol);
      yaw_err = _pids[PID_RATE_YAW].get_pid(degrees(gyro_rate[2]) - (float)in_yaw);
    }    
    // Serial.print(pit_err);
    // Serial.print(" | ");
    // Serial.print(rol_err);
    // Serial.print(" | ");
    // Serial.print(yaw_err);
    // Serial.print(" | ");
    
    _motors.write_pwm(MOTOR_FL, constrain(in_thr + (int16_t)(pit_err - rol_err + yaw_err), 1040, 2000));
    _motors.write_pwm(MOTOR_FR, constrain(in_thr + (int16_t)(pit_err + rol_err - yaw_err), 1040, 2000));
    _motors.write_pwm(MOTOR_RL, constrain(in_thr - (int16_t)(pit_err + rol_err + yaw_err), 1040, 2000));
    _motors.write_pwm(MOTOR_RR, constrain(in_thr - (int16_t)(pit_err - rol_err - yaw_err), 1040, 2000));
    // Serial.print(in_thr + (int16_t)(pit_err - rol_err + yaw_err));
    // Serial.print(" | ");
    // Serial.print(in_thr + (int16_t)(pit_err + rol_err - yaw_err));
    // Serial.print(" | ");
    // Serial.print(in_thr - (int16_t)(pit_err + rol_err + yaw_err));
    // Serial.print(" | ");
    // Serial.print(in_thr - (int16_t)(pit_err - rol_err - yaw_err));
    // Serial.println();
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
  //_auto_level = _radio_prev[CH_AUX1] > 1500 ? true : false;
  if (_radio_prev[CH_AUX1] > 1500) {
    _auto_level = true;
  }
  else {
    _auto_level = false;
  }
  //_auto_level = true;
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
