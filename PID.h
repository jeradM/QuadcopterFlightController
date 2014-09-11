#ifndef __PID_H__
#define __PID_H__

#include "Arduino.h"

class PID {
public:
  PID(float init_p = 0.0, float init_i = 0.0, float init_d = 0.0, uint16_t init_i_limit = 0) {
    _kP            = init_p;
    _kI            = init_i;
    _kD            = init_d;
    _i_limit       = init_i_limit;
    _d_prev        = NAN;
  }
  
  float get_pid(float error);
  
  void operator () (float p, float i, float d, uint16_t i_limit) {
    _kP      = p;
    _kI      = i;
    _kD      = d;
    _i_limit = i_limit;
  }
  
  float get_kP() const {
    return _kP;
  }
  
  float get_kI() const {
    return _kI;
  }
  
  float get_kD() const {
    return _kD;
  }
  
  float get_i_limit() const {
    return _i_limit;
  }
  
  void kP(float p) {
    _kP = p;
  }
  
  void kI(float i) {
    _kI = i;
  }
  
  void kD(float d) {
    _kD = d;
  }
  
  void i_limit(float i) {
    _i_limit = i;
  }
  
  
private: 
  float     _kP;
  float     _kI;
  float     _kD;
  uint16_t  _i_limit;
  float     _i_sum;
  float     _err_prev;
  float     _d_prev;
  uint32_t  _t_prev;
};

#endif
