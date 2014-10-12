#include "FC_Math.h"

float asin_safe(float val) {
  if (isnan(val)) {
    return 0.0f;
  }
  
  if (val >= 1.0f) {
    return PI/2;
  }
  
  if (val <= -1.0f) {
    return -(PI/2);
  }
  
  return asinf(val);
}

float square(float val) {
  if (isnan(val)) {
    return 0;
  }
  
  return val * val;
}

float radians(float deg) {
  if (isnan(deg)) {
    return 0;
  }
  
  return deg * DEG_RAD;
}

float degrees(float rads) {
  if (isnan(rads)) {
    return 0;
  }
  
  return rads * DEG_RAD;
}