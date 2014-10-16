#ifndef _FC_QUATERNION_H_
#define _FC_QUATERNION_H_

#include "Arduino.h"
#include <FC_Math.h>

#define TOLERANCE  0.00001f

class FC_Quaternion {
public:
  FC_Quaternion() {
    w = 1.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
  
  FC_Quaternion(float nw, float nx, float ny, float nz) {
    w = nw;
    x = nx;
    y = ny;
    z = nz;
  }
  
  void update(float *dw, float *a, float dt);
  void update_mahoney(float *dw, float *a, float dt);
  
  FC_Quaternion times(FC_Quaternion q);
  FC_Quaternion plus(FC_Quaternion q);
  FC_Quaternion times_scalar(float s);
  void normalize();
  FC_Quaternion conjugate();
  void to_euler(float *eulers);
  
  float w;
  float x;
  float y;
  float z;
  
};

#endif