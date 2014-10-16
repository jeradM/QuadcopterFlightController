#ifndef __FC_IMU_H__
#define __FC_IMU_H__

#include "Arduino.h"
#include "FC_IMURegisters.h"
#include <FC_Wire.h>
#include <FC_Quaternion.h>
#include <FC_Math.h>

#define FIFO_INT    0

#define ACCEL_LSB_0 16384
#define ACCEL_LSB_1 8192
#define ACCEL_LSB_2 4096
#define ACCEL_LSB_3 2048

#define GYRO_LSB_0  131
#define GYRO_LSB_1  65.5
#define GYRO_LSB_2  32.8
#define GYRO_LSB_3  16.4

extern volatile bool sensor_update_int;

class FC_IMU {
  
public:
  uint32_t _time_prev;
  float _accel_angle[3];
  float _gyro_rate[3];
  int16_t _accel_baseline[3];
  int16_t _gyro_baseline[3];
  int16_t _accel_data[3];
  int16_t _gyro_data[3];
  FC_Quaternion _quaternion;
  
  FC_IMU();
  FC_IMU(uint8_t addr);
  
  // Configuration
  void init();
  bool set_sleep(bool sleep);
  bool set_clock_source(uint8_t clk);
  bool set_clock_divider(uint8_t div);
  bool reset();
  
  // FIFO
  bool fifo_enable(bool en);
  bool fifo_reset();
  bool fifo_enable_data(uint8_t fifo_en_mask = 0x78);
  uint16_t fifo_get_count();
  uint16_t fifo_read(uint8_t num_bytes, uint8_t *data);
  uint16_t fifo_read_packets(uint8_t num_packets, uint16_t *data);
  
  // I2C
  bool i2c_mstr_enable(bool en);
  bool i2c_mstr_reset();
  
  // Interrupts
  bool enable_int_dataready(bool en);
  uint8_t int_status();
  
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
  
  // Latest Sensor Readings
  bool update_sensors();
  void update_quaternion();
  void set_baseline();
  
  int16_t* get_accel() {
    return _accel_data;
  }
  
  int16_t* get_gyro() {
    return _gyro_data;
  }

  
private:
  FC_Wire _tw;
  float _dt;
  uint8_t _i2c_address;


  uint8_t _gyro_fs_sel;
  uint8_t _accel_fs_sel;

  
  bool accel_raw(int16_t *data);
  bool gyro_raw(int16_t *data);
  bool temp_raw(int16_t *data);
  
  bool accel_angle();
  bool gyro_rate();
  bool temp_degrees(int16_t *data);
};

#endif