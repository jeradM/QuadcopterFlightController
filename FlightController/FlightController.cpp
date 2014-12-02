#include "FlightController.h"
#include "FC_IMU.h"

//#define printpiderr
//#define printreadings
//#define printall
//#define printpid

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
  while (!sensor_update_int);
  _imu.update_sensors();
  sensor_update_int = false;
  
  _radio.update(_radio_prev);
  _parse_aux();
  
  long in_thr = map(_radio_prev[CH_THR], _radio._channels_min[CH_THR], _radio._channels_max[CH_THR], 1000, 1850);
  long in_pit, in_rol, in_yaw;

  if (_auto_level) {
    _led_green(true);
    in_pit = -map(_radio_prev[CH_PITCH], _radio._channels_min[CH_PITCH], _radio._channels_max[CH_PITCH], -45, 45);
    in_rol = map(_radio_prev[CH_ROLL], _radio._channels_min[CH_ROLL], _radio._channels_max[CH_ROLL], -45, 45);
    in_yaw = map(_radio_prev[CH_YAW], _radio._channels_min[CH_YAW], _radio._channels_max[CH_YAW], -90, 90);
  }
  else {
    _led_green(false);
    in_pit = -map(_radio_prev[CH_PITCH], _radio._channels_min[CH_PITCH], _radio._channels_max[CH_PITCH], -90, 90);
    in_rol = map(_radio_prev[CH_ROLL], _radio._channels_min[CH_ROLL], _radio._channels_max[CH_ROLL], -90, 90);
    in_yaw = map(_radio_prev[CH_YAW], _radio._channels_min[CH_YAW], _radio._channels_max[CH_YAW], -90, 90);
  }

  float *gyro_rate = _imu.get_gyro();
  
  if (in_thr > 1100 && _motors.is_armed()) {
    double pit_err, rol_err, yaw_err;
    if (_auto_level) {
      _imu._quaternion.to_euler(eulers);
      pit_stab_err = constrain(_pids[PID_STAB_PIT].get_pid((float)in_pit - degrees(eulers[1])), -250, 250);
      rol_stab_err = constrain(_pids[PID_STAB_ROL].get_pid((float)in_rol - degrees(eulers[0])), -250, 250);
      yaw_stab_err = constrain(_pids[PID_STAB_YAW].get_pid(wrap_180((float)yaw_target - degrees(eulers[2]))), -360, 360);
	
//****************************************************************//			
#if defined (printreadings)
			if (cnt == 20) {
				Serial.print(in_pit);
				Serial.print(":");
				Serial.print(degrees(eulers[1]));
				Serial.print(" | ");
				Serial.print(in_rol);
				Serial.print(":");
				Serial.print(degrees(eulers[0]));
				Serial.print(" | ");
				Serial.print(in_yaw);
				Serial.print(":");
				Serial.print(degrees(eulers[2]));
				Serial.println();
				cnt = 0;
			}
			else {
				cnt++;
			}
			
#endif
//****************************************************************//	
			
			if (abs(in_yaw) > 10) {
				yaw_stab_err = in_yaw;
				yaw_target = degrees(eulers[2]);
			}
      
      pit_err = constrain(_pids[PID_RATE_PIT].get_pid(degrees(gyro_rate[1]) - pit_stab_err), -500, 500);
      rol_err = constrain(_pids[PID_RATE_ROL].get_pid(degrees(gyro_rate[0]) - rol_stab_err), -500, 500);
      yaw_err = constrain(_pids[PID_RATE_YAW].get_pid(degrees(gyro_rate[2]) - yaw_stab_err), -500, 500);
	
//****************************************************************//		
#if defined (printall)
		if (cnt == 20) {
			Serial.print(pit_stab_err);
			Serial.print(":");
			Serial.print(degrees(gyro_rate[1]));
			Serial.print(":");
			Serial.print(pit_err);
			Serial.print(" | ");
			Serial.print(rol_stab_err);
			Serial.print(":");
			Serial.print(degrees(gyro_rate[0]));
			Serial.print(":");
			Serial.print(rol_err);
			Serial.print(" | ");
			Serial.print(yaw_stab_err);
			Serial.print(":");
			Serial.print(degrees(gyro_rate[2]));
			Serial.print(":");
			Serial.print(yaw_err);
			Serial.println();
			cnt = 0;
		}
		else {
			cnt++;
		}
#endif
#if defined (printpid)
		if (cnt == 20) {
			_pids[PID_RATE_YAW].debug = true;
			cnt = 0;
		}
		else {
			_pids[PID_RATE_YAW].debug = false;
			cnt++;
		}
#endif
//****************************************************************//	
    }
    else {
      pit_err = constrain(_pids[PID_RATE_PIT].get_pid(degrees(gyro_rate[1]) - (float)in_pit), -500, 500);
      rol_err = constrain(_pids[PID_RATE_ROL].get_pid(degrees(gyro_rate[0]) - (float)in_rol), -500, 500);
      yaw_err = constrain(_pids[PID_RATE_YAW].get_pid(degrees(gyro_rate[2]) - (float)in_yaw), -500, 500);
		
//****************************************************************//		
#if defined (printall)
		if (cnt == 20) {
			Serial.print(in_pit);
			Serial.print(":");
			Serial.print(degrees(gyro_rate[1]));
			Serial.print(":");
			Serial.print(pit_err);
			Serial.print(" | ");
			Serial.print(in_rol);
			Serial.print(":");
			Serial.print(degrees(gyro_rate[0]));
			Serial.print(":");
			Serial.print(rol_err);
			Serial.print(" | ");
			Serial.print(in_yaw);
			Serial.print(":");
			Serial.print(degrees(gyro_rate[2]));
			Serial.print(":");
			Serial.print(yaw_err);
			Serial.println();
			cnt = 0;
		}
		else {
			cnt++;
		}
#endif
//****************************************************************//	
		
    }  
		
//****************************************************************//	
#if defined (printpiderr)
		if (cnt == 20) {
			Serial.print(pit_err, 4);
			Serial.print(" | ");
			Serial.print(rol_err, 4);
			Serial.print(" | ");
			Serial.print(yaw_err, 4);
			Serial.println();	  
			cnt = 0;
		}
		else {
			cnt++;
		}
#endif
//****************************************************************//	
		
    _motors.write_pwm(MOTOR_FL, constrain(in_thr + (int16_t)(pit_err - rol_err - yaw_err), 1100, 2000));
    _motors.write_pwm(MOTOR_FR, constrain(in_thr + (int16_t)(pit_err + rol_err + yaw_err), 1100, 2000));
    _motors.write_pwm(MOTOR_RL, constrain(in_thr - (int16_t)(pit_err + rol_err - yaw_err), 1100, 2000));
    _motors.write_pwm(MOTOR_RR, constrain(in_thr - (int16_t)(pit_err - rol_err + yaw_err), 1100, 2000));

  }
  else {
    _motors.write_pwm(MOTOR_FL, 1000);
    _motors.write_pwm(MOTOR_FR, 1000);
    _motors.write_pwm(MOTOR_RL, 1000);
    _motors.write_pwm(MOTOR_RR, 1000);
		yaw_target = eulers[2];
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
		if (reset_quat) {
			_imu._quaternion.reset();
			reset_quat = false;
		}
  }
  else {
    _auto_level = false;
		if (!reset_quat) {
			reset_quat = true;
		}
  }
  //_auto_level = true;
}

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
