#ifndef __FLIGHT_CONTROLLER_H__
#define __FLIGHT_CONTROLLER_H__

#include "MotorOutput.h"
#include "RadioInput.h"
#include "PID.h"

class FlightController {
public:
  FlightController() {}
  
  bool armed = false;
  
  void init();
  void arm();
  void disarm();
  void calibrate_radio();
  
  
private:
  MotorOutput motors;
  RadioInput  radio;
  
};


#endif