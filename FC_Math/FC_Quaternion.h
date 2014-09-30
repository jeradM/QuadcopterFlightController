#ifndef _FC_QUATERNION_H_
#define _FC_QUATERNION_H_

#import "Arduino.h"

#define TOLERANCE  0.00001f

class FC_Quaternion {
public:
  FC_Quaternion() {
    _w = 0.0;
    _x = 0.0;
    _y = 0.0;
    _z = 1.0;
  }
  
  FC_Quaternion(nw, nx, ny, nz) {
    _w = nw;
    _x = nx;
    _y = ny;
    _z = nz;
  }
  
  FC_Quaternion operator+(const FC_Quaternion &q1);
  FC_Quaternion cross_with(FC_Quaternion q);
  FC_Quaternion times_scalar(float s);
  void normailze();
  FC_Quaternion conjugate();
  FC_Quaternion rate_derivative(float rx, float ry, float rz);
  
  float _w;
  float _x;
  float _y;
  float _z;
  
private:
};

#endif