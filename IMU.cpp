#include "IMU.h"
#include "TwoWire.h"

// Set I2C address and init I2C
void IMU::init(uint8_t slv_addr) {
  _i2c_address = slv_addr;
  Wire.begin(_i2c_address);
}

// Set Gyroscope Sensitivity (resolution)
// fs_sel  - Range selector
// returns - true on successful write
// Gyro (deg/s): (0: 250 | 1: 500 | 2: 1000 | 3: 2000)
bool gyro_set_range(uint8_t fs_sel) {
  return tw.write_bits(_i2c_address, GYRO_CONFIG, 4, 2, &fs_sel);
}

// Set Accelerometer Sensitivity (resolution)
// fs_sel  - Range selector
// returns - true on successful write
// Accel (Gs): (0: 2 | 1: 4 | 2: 8 | 3: 16)
bool accel_set_range(uint8_t fs_sel) {
  return tw.write_bits(_i2c_address, ACCEL_CONFIG, 4, 2, &fs_sel);
}

// Read raw accelerometer values
// data     - input buffer to store values [x, y, z]
// returns  - true on success
bool read_accel_raw(uint16_t *data) {
  uint8_t tmp_bytes[6];
  
  if (tw.read_bytes(_i2c_address, ACCEL_XOUT_H, 6, &tmp_bytes)) {
    // Accel X - H/L bytes
    data[0] = tmp_bytes[0];
    data[0] = (data[0] << 8) | tmp_bytes[1];
    // Accel Y
    data[1] = tmp_bytes[2];
    data[1] = (data[1] << 8) | tmp_bytes[3];
    // Accel Z
    data[2] = tmp_bytes[4];
    data[2] = (data[2] << 8) | tmp_bytes[5];
    return true;
  }
  
  return false;
}

// Read raw gyroscope values
// data     - input buffer to store values [x, y, z]
// returns  - true on success
bool read_gyro_raw(uint16_t *data) {
  uint8_t tmp_bytes[6];
  
  if (tw.read_bytes(_i2c_address, GYRO_XOUT_H, 6, &tmp_bytes)) {
    //Gyro X - H/l bytes
    data[0] = tmp_bytes[0];
    data[0] = (data[0] << 8) | tmp_bytes[1];
    // Gyro Y
    data[1] = tmp_bytes[2];
    data[1] = (data[1] << 8) | tmp_bytes[3];
    // Gyro Z
    data[2] = tmp_bytes[4];
    data[2] = (data[2] << 8) | tmp_bytes[5];
    
    return true;
  }
  
  return false;
}

// Read raw temperature values
// data     - input buffer to store value
// returns  - true on success
bool read_temp_raw(uint16_t *data) {
  uint8_t tmp_bytes[2];
  
  if (tw.read_bytes(_i2c_address, TEMP_OUT_H, 2, &tmp_bytes)) {
    data = tmp_bytes[0];
    data = (data << 8) | tmp_bytes[1];
    
    return true;
  }
  
  return false;
}
