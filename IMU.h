#ifndef __IMU_H__
#define __IMU_H__

#include <Wire.h>
#include "IMURegisters.h"

class IMU {
  
public:
  IMU();
  
  void init(uint8_t slv_addr);
  
  void set_sleep(bool sleep);
  
  // Set sensitivity for sensors
  bool gyro_set_range(uint8_t fs_sel);
  bool accel_set_range(uint8_t fs_sel);
  
  // FIFO enable methods
  bool fifo_enable_gyro();
  bool fifo_enable_accel();
  bool fifo_enable_temp();
  bool fifo_enable_slv(uint8_t slv_mask); // 3 bit slv_msk | 2 | 1 | 0 |
  bool fifo_enable_mask(uint8_t mask);
  bool fifo_enable_all();
  
  // Raw sensor values
  bool read_accel_raw(uint16_t *data);
  bool read_gyro_raw(uint16_t *data);
  bool read_temp_raw(uint16_t *data);
  
private:
  uint8_t _i2c_address;
  TwoWire tw;
};

#endif