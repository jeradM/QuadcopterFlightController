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
  delay(250);
  _motors.init();
  //delay(250);
  
  _radio.update(_radio_prev);
  if (_radio_prev[CH_THR] > _radio._channels_max[CH_THR] - 100) {
    _esc_calibration();
  }
  
  while (!_initialized) {
    _radio.update(_radio_prev);
    if (_radio_prev[CH_THR] < _radio._channels_min[CH_THR] + 100) {
      _initialized = true;
    }
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
		// Map radio channels to desired attitude angles
    in_pit = -map(_radio_prev[CH_PITCH], _radio._channels_min[CH_PITCH], _radio._channels_max[CH_PITCH], -45, 45);
    in_rol = map(_radio_prev[CH_ROLL], _radio._channels_min[CH_ROLL], _radio._channels_max[CH_ROLL], -45, 45);
    in_yaw = -map(_radio_prev[CH_YAW], _radio._channels_min[CH_YAW], _radio._channels_max[CH_YAW], -150, 150);
  }
  else {
    _led_green(false);
		// Map radio channels to desired rotational velocities
    in_pit = -map(_radio_prev[CH_PITCH], _radio._channels_min[CH_PITCH], _radio._channels_max[CH_PITCH], -90, 90);
    in_rol = map(_radio_prev[CH_ROLL], _radio._channels_min[CH_ROLL], _radio._channels_max[CH_ROLL], -90, 90);
    in_yaw = -map(_radio_prev[CH_YAW], _radio._channels_min[CH_YAW], _radio._channels_max[CH_YAW], -150, 150);
  }

	// Read current 3-axis rotational rates from gyroscope
  float *gyro_rate = _imu.get_gyro();
  
  if (in_thr > 1100 && _motors.is_armed()) {
    float pit_err, rol_err, yaw_err;
    if (_auto_level) { // Auto-Level enabled - Perform Stability PID Calculations
      float eulers[3];
      _imu._quaternion.to_euler(eulers);
      float pit_stab_err = _pids[PID_STAB_PIT].get_pid((float)in_pit - degrees(eulers[1]));
      float rol_stab_err = _pids[PID_STAB_ROL].get_pid((float)in_rol - degrees(eulers[0]));
      float yaw_stab_err = _pids[PID_STAB_YAW].get_pid((float)in_yaw - degrees(eulers[2]));
      
      pit_err = _pids[PID_RATE_PIT].get_pid(degrees(gyro_rate[1]) - pit_stab_err);
      rol_err = _pids[PID_RATE_ROL].get_pid(degrees(gyro_rate[0]) - rol_stab_err);
      yaw_err = _pids[PID_RATE_YAW].get_pid(degrees(gyro_rate[2]) - yaw_stab_err);
    }
    else { // Rate mode enabled - perform rate PID calculations
      pit_err = _pids[PID_RATE_PIT].get_pid(degrees(gyro_rate[1]) - (float)in_pit);
      rol_err = _pids[PID_RATE_ROL].get_pid(degrees(gyro_rate[0]) - (float)in_rol);
      yaw_err = _pids[PID_RATE_YAW].get_pid(degrees(gyro_rate[2]) - (float)in_yaw);
    }    
    
		// Set PWM of each motor based on PID controller outputs
    _motors.write_pwm(MOTOR_FL, constrain(in_thr + (int16_t)(pit_err - rol_err + yaw_err), 1040, 2000));
    _motors.write_pwm(MOTOR_FR, constrain(in_thr + (int16_t)(pit_err + rol_err - yaw_err), 1040, 2000));
    _motors.write_pwm(MOTOR_RL, constrain(in_thr - (int16_t)(pit_err + rol_err + yaw_err), 1040, 2000));
    _motors.write_pwm(MOTOR_RR, constrain(in_thr - (int16_t)(pit_err - rol_err - yaw_err), 1040, 2000));
		
  }
  else {
		// Throttle in MIN position - turn off all motors
    _motors.write_pwm(MOTOR_FL, 1000);
    _motors.write_pwm(MOTOR_FR, 1000);
    _motors.write_pwm(MOTOR_RL, 1000);
    _motors.write_pwm(MOTOR_RR, 1000);
    if (in_thr < _radio._channels_min[CH_THR] + 30) {
      if (_radio_prev[CH_YAW] > _radio._channels_max[CH_YAW] - 50 && _motors.is_armed()) {
				// Arm motors
        _motors.disarm();
        _led_blue(false);
      }
      else if (_radio_prev[CH_YAW] < _radio._channels_min[CH_YAW] + 50 && !_motors.is_armed()) {
				// Disarm motors
        _motors.arm();
        _led_blue(true);
      }
    }
  }
  
  
}

// Check for auto level mode based on yaw stick
void FlightController::_parse_aux() {
  //_auto_level = _radio_prev[CH_AUX1] > 1500 ? true : false;
  if (_radio_prev[CH_AUX1] > 1500) {
    _auto_level = true;
  }
  else {
    _auto_level = false;
  }
}

// Start ESC calibration
void FlightController::_esc_calibration() {
  _motors.arm();
  for (int i = 0; i < 4; i++) {
    _motors.write_pwm(i, 2000);
  }
  
  while (true) {
    _radio.update(_radio_prev);
    if (_radio_prev[CH_THR] < _radio._channels_min[CH_THR] + 50) {
      for (int i = 0; i < 4; i++) {
        _motors.write_pwm(i, 1000);
      }
    }
  }
  
  while(true);
}

// Initalize PIDs with starting values
void FlightController::_setup_pids() {
  _pids[PID_RATE_ROL](PID_RATE_ROL_P, PID_RATE_ROL_I, PID_RATE_ROL_D, PID_RATE_ROL_IL);
  _pids[PID_RATE_PIT](PID_RATE_PIT_P, PID_RATE_PIT_I, PID_RATE_PIT_D, PID_RATE_PIT_IL);
  _pids[PID_RATE_YAW](PID_RATE_YAW_P, PID_RATE_YAW_I, PID_RATE_YAW_D, PID_RATE_YAW_IL);
  _pids[PID_STAB_ROL](PID_STAB_ROL_P, PID_STAB_ROL_I, PID_STAB_ROL_D, PID_STAB_ROL_IL);
  _pids[PID_STAB_PIT](PID_STAB_PIT_P, PID_STAB_PIT_I, PID_STAB_PIT_D, PID_STAB_PIT_IL);
  _pids[PID_STAB_YAW](PID_STAB_YAW_P, PID_STAB_YAW_I, PID_STAB_YAW_D, PID_STAB_YAW_IL);
}

// Blue LED - Motors Armed Indicator
void FlightController::_led_blue(bool on) {
  if (on) {
    PORTB |= (1 << LED_BLUE);
  }
  else {
    PORTB &=~ (1 << LED_BLUE);
  }
}

// Green LED - Auto-Level mode indication
void FlightController::_led_green(bool on) {
  if (on) {
    PORTB |= (1 << LED_GREEN);
  }
  else {
    PORTB &=~ (1 << LED_GREEN);
  }
}
