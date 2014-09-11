 #ifndef __MOTOR_OUTPUT_H__
 #define __MOTOR_OUTPUT_H__
 
 #include "Arduino.h"
 
 class MotorOutput {
 public:
   MotorOutput(int num_motors = 4) {
     _num_motors = num_motors;
   }
   
   void init_motors();
   void write_pwm(int channel, uint16_t in_pulse);
   
   // ----------------------------------------
   // Setters/Getters for min/max pulse widths
   void min_pulse(uint16_t p) {
     _min_pulse = p >> 4;
   }
   
   void max_pulse(uint16_t p) {
     _max_pulse = p >> 4;
   }
   
   uint16_t min_pulse() const {
     return _min_pulse;
   }
   
   uint16_t max_pulse() const {
     return _max_pulse;
   }
   // ----------------------------------------
   
 private:
   bool _armed = false;
   int _num_motors = 4;
   uint16_t _min_pulse = 62;
   uint16_t _max_pulse = 125;
   
   uint8_t _constrain_pwm(uint16_t pwm);
 };
 
 #endif