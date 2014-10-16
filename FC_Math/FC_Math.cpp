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
  
  return asin(val);
}