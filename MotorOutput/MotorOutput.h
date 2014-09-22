 #ifndef __MOTOR_OUTPUT_H__
 #define __MOTOR_OUTPUT_H__
 
 #include "Arduino.h"
 
 #define MOTOR_FL   0
 #define MOTOR_FR   1
 #define MOTOR_RR   2
 #define MOTOR_RL   3
 
 class MotorOutput {
 public:
   MotorOutput(int num_motors = 4) {
     _num_motors = num_motors;
     _min_pulse = 62;
     _max_pulse = 125;
     _esc_init = false;
     _armed = false;
   }
   
   void init();
   void init_esc();
   void write_pwm(int channel, uint16_t in_pulse);
   
   void arm();
   void disarm();
   
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
   int _num_motors;
   uint16_t _min_pulse;
   uint16_t _max_pulse;
   bool _esc_init;
   bool _armed;
   
   uint8_t _constrain_pwm(uint16_t pwm);
 };
 
 #endif