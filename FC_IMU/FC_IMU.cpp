#include <Wire.h>
#include "FC_IMU.h"

FC_IMU::FC_IMU() {
  
}

// Set I2C address and init I2C
void FC_IMU::init(uint8_t slv_addr) {
  _i2c_address = slv_addr;
  Wire.begin(_i2c_address);
}

// Set sleep mode on or off
bool FC_IMU::set_sleep(bool on) {
  
}

// Set Gyroscope Sensitivity (resolution)
// fs_sel  - Range selector
// returns - true on successful write
// Gyro (deg/s): (0: 250 | 1: 500 | 2: 1000 | 3: 2000)
bool FC_IMU::gyro_set_range(uint8_t fs_sel) {
  _gyro_fs_sel = fs_sel;
  return tw.write_bits(_i2c_address, GYRO_CONFIG, 4, 2, fs_sel);
}

// Set Accelerometer Sensitivity (resolution)
// fs_sel  - Range selector
// returns - true on successful write
// Accel (Gs): (0: 2 | 1: 4 | 2: 8 | 3: 16)
bool FC_IMU::accel_set_range(uint8_t fs_sel) {
  _accel_fs_sel = fs_sel;
  return tw.write_bits(_i2c_address, ACCEL_CONFIG, 4, 2, fs_sel);
}

// Calculate accelerometer angles
// data     - buffer to store angles
// returns  - true on successful read
bool FC_IMU::accel_angle(int16_t *data) {
  
}

// Read raw accelerometer values
// data     - input buffer to store values [x, y, z]
// returns  - true on success
bool FC_IMU::accel_raw(int16_t *data) {
  uint8_t tmp_bytes[6];
  
  if (tw.read_bytes(_i2c_address, ACCEL_XOUT_H, 6, tmp_bytes)) {
    // Accel X - H/L bytes
    data[0] = (( (uint16_t)(0 | tmp_bytes[0]) ) << 8) | tmp_bytes[1];
    // Accel Y
    data[1] = (( (uint16_t)(0 | tmp_bytes[2]) ) << 8) | tmp_bytes[3];
    // Accel Z
    data[2] = (( (uint16_t)(0 | tmp_bytes[4]) ) << 8) | tmp_bytes[5];
    return true;
  }
  
  return false;
}

// Read raw gyroscope values
// data     - input buffer to store values [x, y, z]
// returns  - true on success
bool FC_IMU::gyro_raw(int16_t *data) {
  uint8_t tmp_bytes[6];
  
  if (tw.read_bytes(_i2c_address, GYRO_XOUT_H, 6, tmp_bytes)) {
    //Gyro X - H/L bytes
    data[0] = (( (uint16_t)(0 | tmp_bytes[0]) ) << 8) | tmp_bytes[1];
    // Gyro Y
    data[1] = (( (uint16_t)(0 | tmp_bytes[2]) ) << 8) | tmp_bytes[3];
    // Gyro Z
    data[2] = (( (uint16_t)(0 | tmp_bytes[4]) ) << 8) | tmp_bytes[5];
    
    return true;
  }
  
  return false;
}

// Read raw temperature values
// data     - input buffer to store value
// returns  - true on success
bool FC_IMU::temp_raw(int16_t *data) {
  uint8_t tmp_bytes[2];
  
  if (tw.read_bytes(_i2c_address, TEMP_OUT_H, 2, tmp_bytes)) {
    data[0] = (( (uint16_t)(0 | tmp_bytes[0]) ) << 8) | tmp_bytes[1];
    
    return true;
  }
  
  return false;
}
