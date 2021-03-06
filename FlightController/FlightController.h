#ifndef __FLIGHT_CONTROLLER_H__
#define __FLIGHT_CONTROLLER_H__

#include <MotorOutput.h>
#include <RadioInput.h>
#include <PID.h>
#include <FC_IMU.h>

#define LED_BLUE            4
#define LED_GREEN           5

#define PID_RATE_ROL        0
  #define PID_RATE_ROL_P   0.7
  #define PID_RATE_ROL_I   0.4
  #define PID_RATE_ROL_D   0.0
  #define PID_RATE_ROL_IL  0.9
  
#define PID_RATE_PIT        1
  #define PID_RATE_PIT_P    0.7
  #define PID_RATE_PIT_I    0.4
  #define PID_RATE_PIT_D    0.0
  #define PID_RATE_PIT_IL   0.9
  
#define PID_RATE_YAW        2
  #define PID_RATE_YAW_P    1.0
  #define PID_RATE_YAW_I    0.3
  #define PID_RATE_YAW_D    0.0
  #define PID_RATE_YAW_IL   1.0
  
#define PID_STAB_ROL        3
  #define PID_STAB_ROL_P    2.5
  #define PID_STAB_ROL_I    0.4
  #define PID_STAB_ROL_D    0.0
  #define PID_STAB_ROL_IL   1.0
  
#define PID_STAB_PIT        4
  #define PID_STAB_PIT_P    2.5
  #define PID_STAB_PIT_I    0.4
  #define PID_STAB_PIT_D    0.0
  #define PID_STAB_PIT_IL   1.0
  
#define PID_STAB_YAW        5
  #define PID_STAB_YAW_P    4.5
  #define PID_STAB_YAW_I    0.4
  #define PID_STAB_YAW_D    0.0
  #define PID_STAB_YAW_IL   1.0

#define wrap_180(x) (x < -180 ? x + 360 : (x > 180 ? x -360 : x))

class FlightController {
public:
  FC_IMU _imu;
  MotorOutput _motors;
  RadioInput  _radio;
  
  FlightController() {}
  
  void init();
  void calibrate_radio();
  
  // Update Inputs/Outputs/Auxillary
  // Call in main program loop
  void update();
  
  
private:
  PID _pids[6];
	
	int cnt;
	
	double pit_stab_err;
	double rol_stab_err;
	double yaw_stab_err;
	float yaw_target;
  float eulers[3];
  
  int16_t _accel[3];
  int16_t _gyro[3];
  uint16_t _radio_prev[8];
  uint16_t _motor_prev[4];
  
  bool _initialized;
  bool _auto_level;
	
	bool reset_quat;
  
  // Auxillary Input Functions
  void _parse_aux();
  void _esc_calibration();
  void _setup_pids();
  void _led_blue(bool on);
  void _led_green(bool on);
  
};


#endif