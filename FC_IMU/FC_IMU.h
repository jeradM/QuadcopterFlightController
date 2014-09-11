#ifndef __FC_IMU_H__
#define __FC_IMU_H__

#include "Arduino.h"
#include "FC_IMURegisters.h"
#include <FC_Wire.h>

#define ACCEL_LSB_0 16384
#define ACCEL_LSB_1 8192
#define ACCEL_LSB_2 4096
#define ACCEL_LSB_3 2048

#define GYRO_LSB_0  131
#define GYRO_LSB_1  65.5
#define GYRO_LSB_2  32.8
#define GYRO_LSB_3  16.4

class FC_IMU {
  
public:
  FC_IMU();
  
  void init(uint8_t slv_addr);
  
  bool set_sleep(bool sleep);
  
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
  bool accel_raw(int16_t *data);
  bool gyro_raw(int16_t *data);
  bool temp_raw(int16_t *data);
  
  // Read Calculated sensor values
  bool accel_angle(float *data);
  bool gyro_rate(float *data);
  bool temp_degrees(int16_t *data);
  
private:
  FC_Wire tw;
  uint8_t _i2c_address;
  uint8_t _gyro_fs_sel;
  uint8_t _accel_fs_sel;
};

#endif