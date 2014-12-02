#include "PID.h"

#include <math.h>

float PID::get_pid(float error) {
  uint32_t cur_time = millis();
  uint32_t dt       = cur_time - _t_prev;
  float result      = 0.0;
  float time_dif;
   
   if (_t_prev == 0 || dt > 1000) {
     dt = 0;
     _i_sum = 0;
   }
	 
	 if (debug) {
		 Serial.print("Error: ");
		 Serial.print(error);
		 Serial.print(" | dt: ");
		 Serial.print(dt);
	 }
   
   _t_prev = cur_time;
   time_dif = (float)dt/1000.0f;
   
   result += error * _kP;
	 
	 if (debug) {
		 Serial.print(" | ");
		 Serial.print("P Error: ");
		 Serial.print(result);
	 }
   
   if (abs(_kI) > 0 && dt > 0) {
     _i_sum += error * _kI * time_dif;
     
     if (_i_sum < -_i_limit) _i_sum     = -_i_limit;
     else if (_i_sum > _i_limit) _i_sum = _i_limit;  
     
     result += _i_sum;	 
		 
		 if (debug) {
			 Serial.print(" | ");
			 Serial.print("I Error: ");
			 Serial.print(result);
		 }
		 
   }
   
   if (abs(_kD) > 0 && dt > 0) {
     float dx;
     
     if (isnan(_d_prev)) {
       dx      = 0.0f;
       _d_prev = 0.0f;
     }
     else {
       dx = (error - _err_prev) / time_dif;
     }
     
     result += dx * _kD;
		 
		 if (debug) {
			 Serial.print(" | ");
			 Serial.print("D Error: ");
			 Serial.print(result);
		 }
		 
   }
	 
	 if (debug) {
	   Serial.print(" | Error: ");
		 Serial.println(result);
	 }
	 
   return result;
}
