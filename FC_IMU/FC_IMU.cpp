#include "FC_IMU.h"

volatile bool sensor_update_int = false;

FC_IMU::FC_IMU() {
  _i2c_address = 0x68;
}

FC_IMU::FC_IMU(uint8_t addr) {
  _i2c_address = addr;
}

// Set I2C address and init I2C
void FC_IMU::init() {
  _quaternion = FC_Quaternion();
  set_clock_source(0x01);
  gyro_set_range(0x00);
  accel_set_range(0x00);
  set_sleep(false);
  set_baseline();
  TIMSK2 |= (1 << TOIE2);
  update_sensors();
}

// Set sleep mode on or off
bool FC_IMU::set_sleep(bool on) {
  if (on) {
    return _tw.write_bit(_i2c_address, PWR_MGMT_1, 6, 1);
  }
  else {
    return _tw.write_bit(_i2c_address, PWR_MGMT_1, 6, 0);
  }
}

// Set clock source
bool FC_IMU::set_clock_source(uint8_t clk) {
  return _tw.write_bits(_i2c_address, PWR_MGMT_1, 2, 3, clk);
}

// Set Sample Rate Clock Divider
bool FC_IMU::set_clock_divider(uint8_t div) {
  return _tw.write_byte(_i2c_address, SMPRT_DIV, div);
}

// Set board reset bit - reset all registers
bool FC_IMU::reset() {
  return _tw.write_bit(_i2c_address, PWR_MGMT_1, 7, 1);
}

// Enable FIFO Register
bool FC_IMU::fifo_enable(bool en) {
  uint8_t bv = 0;
  if (en) bv = 1;
  return _tw.write_bit(_i2c_address, USER_CTRL, 6, bv);
}

// Reset FIFO Buffer
bool FC_IMU::fifo_reset() {
  return _tw.write_bit(_i2c_address, USER_CTRL, 2, 1);
}

// Enable i2C Master Mode
bool FC_IMU::i2c_mstr_enable(bool en) {
  uint8_t bv = 0;
  if (en) bv = 1;
  return _tw.write_bit(_i2c_address, USER_CTRL, 5, bv);
}

// i2C Master Reset
bool FC_IMU::i2c_mstr_reset() {
  return _tw.write_bit(_i2c_address, USER_CTRL, 1, 1);
}

// Set which data to send to FIFO_R_W
// fifo_en_mask  - Bitmask to set enabled register (see FC_IMU_Registers.h)
//                 (defaults to gyro and accel only)
bool FC_IMU::fifo_enable_data(uint8_t fifo_en_mask) {
  return _tw.write_byte(_i2c_address, FIFO_EN, fifo_en_mask);
}

// Get sample count from FIFO buffer
uint16_t FC_IMU::fifo_get_count() {
  uint8_t tmp_cnt[2];
  _tw.read_bytes(_i2c_address, FIFO_COUNT_H, 2, tmp_cnt);
  return (( (uint16_t)tmp_cnt[0] )  << 8) | tmp_cnt[1];
}

// Read bytes from FIFO buffer
// num_bytes   - number of bytes to read from fifo
//
uint16_t FC_IMU::fifo_read(uint8_t num_bytes, uint8_t *data) {
  return _tw.read_bytes(_i2c_address, FIFO_R_W, num_bytes, data);
}

// Read FIFO data 16-bits at a time
uint16_t FC_IMU::fifo_read_packets(uint8_t num_packets, uint16_t *data) {
  return _tw.read_packets(_i2c_address, FIFO_R_W, num_packets, data);
}

// Enable DataReady interrupt
bool FC_IMU::enable_int_dataready(bool en) {
  uint8_t b = 0;
  if (en) b = 1;
  return _tw.write_bit(_i2c_address, INT_ENABLE, 0, b);
}

uint8_t FC_IMU::int_status() {
  uint8_t data;
  _tw.read_byte(_i2c_address, INT_STATUS, &data);
  return data;
}

// Set Gyroscope Sensitivity (resolution)
// fs_sel  - Range selector
// returns - true on successful write
// Gyro (deg/s): (0: 250 | 1: 500 | 2: 1000 | 3: 2000)
bool FC_IMU::gyro_set_range(uint8_t fs_sel) {
  _gyro_fs_sel = fs_sel;
  return _tw.write_bits(_i2c_address, GYRO_CONFIG, 4, 2, fs_sel);
}

// Set Accelerometer Sensitivity (resolution)
// fs_sel  - Range selector
// returns - true on successful write
// Accel (Gs): (0: 2 | 1: 4 | 2: 8 | 3: 16)
bool FC_IMU::accel_set_range(uint8_t fs_sel) {
  _accel_fs_sel = fs_sel;
  return _tw.write_bits(_i2c_address, ACCEL_CONFIG, 4, 2, fs_sel);
}

// Update Sensor Readings into _latest_sensor_data
bool FC_IMU::update_sensors() {
  uint8_t data[14];
  _tw.read_bytes(_i2c_address, ACCEL_XOUT_H, 14, data);
  
  _accel_data[0]  =  ((((int16_t)data[0])  << 8) | data[1])  - _accel_baseline[0];
  _accel_data[1]  =  ((((int16_t)data[2])  << 8) | data[3])  - _accel_baseline[1];
  _accel_data[2]  =  ((((int16_t)data[4])  << 8) | data[5])  - _accel_baseline[2] + (int16_t)16384;
  _gyro_data[0]   =  ((((int16_t)data[8])  << 8) | data[9])  - _gyro_baseline[0];
  _gyro_data[1]   =  ((((int16_t)data[10]) << 8) | data[11]) - _gyro_baseline[1];
  _gyro_data[2]   =  ((((int16_t)data[12]) << 8) | data[13]) - _gyro_baseline[2]; 
  
  gyro_rate();
  accel_angle();
  
  update_quaternion();
}

void FC_IMU::update_quaternion() {
  if (_time_prev == 0) {
    _time_prev = millis();
  }
  else {
    uint32_t time = millis();
    _dt = (float)(time - _time_prev);
    _time_prev = time;
    _dt /= 1000.0f;
    _quaternion.update_mahoney(_gyro_rate, _accel_angle, _dt); 
  }
}

// Set baseline MPU readings at init
void FC_IMU::set_baseline() {
  int16_t tmp_acc[3] = {0, 0, 0};
  int16_t tmp_gyro[3] = {0, 0, 0};
  
  int32_t acc_tot[3] = {0, 0, 0};
  int32_t gyro_tot[3] = {0, 0, 0};
  
  for (int i = 20; i --> 0;) {
    accel_raw(tmp_acc);
    gyro_raw(tmp_gyro);
    
    for (int j = 0; j < 3; j++) {
      acc_tot[j]  += (int32_t)tmp_acc[j];
      gyro_tot[j] += (int32_t)tmp_gyro[j];
    }
    
    delay(100);
  }
  
  for (int i = 0; i < 3; i++) {
    _accel_baseline[i] = (int16_t)(acc_tot[i] / 20);
    _gyro_baseline[i]  = (int16_t)(gyro_tot[i] / 20);
  }
}

// Calculate accelerometer angles
// data     - buffer to store angles
// returns  - true on successful read
bool FC_IMU::accel_angle() {
  float alsb;
  
  switch(_accel_fs_sel) {
    case 0:   alsb = (float)ACCEL_LSB_0; break;
    case 1:   alsb = (float)ACCEL_LSB_1; break;
    case 2:   alsb = (float)ACCEL_LSB_2; break;
    case 3:   alsb = (float)ACCEL_LSB_3; break;
    default:  alsb = (float)ACCEL_LSB_0; break;
  }
  
  for (int i = 0; i < 3; i++) {
    _accel_angle[i] = ((float)_accel_data[i])/alsb;
  }
  //_accel_angle[0] *= -1.0f;
  // _accel_angle[1] *= -1.0f;
  // _accel_angle[2] *= -1.0f;
}

bool FC_IMU::gyro_rate() {
  float glsb;
  
  switch(_gyro_fs_sel) {
    case 0:   glsb = (float)GYRO_LSB_0; break;
    case 1:   glsb = (float)GYRO_LSB_1; break;
    case 2:   glsb = (float)GYRO_LSB_2; break;
    case 3:   glsb = (float)GYRO_LSB_3; break;
    default:  glsb = (float)GYRO_LSB_0;
  }
  
  for (int i = 0; i < 3; i++) {
    _gyro_rate[i] = radians(((float)_gyro_data[i])/glsb);
  }
  //_gyro_rate[1] *= -1.0f;
}

// Read raw accelerometer values
// data     - input buffer to store values [x, y, z]
// returns  - true on success
bool FC_IMU::accel_raw(int16_t *data) {
  uint8_t tmp_bytes[6];
  
  if (_tw.read_bytes(_i2c_address, ACCEL_XOUT_H, 6, tmp_bytes)) {
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
  
  if (_tw.read_bytes(_i2c_address, GYRO_XOUT_H, 6, tmp_bytes)) {
    //Gyro X - H/L bytes
    data[0] = (( (int16_t)(0 | tmp_bytes[0]) ) << 8) | tmp_bytes[1];
    // Gyro Y
    data[1] = (( (int16_t)(0 | tmp_bytes[2]) ) << 8) | tmp_bytes[3];
    // Gyro Z
    data[2] = (( (int16_t)(0 | tmp_bytes[4]) ) << 8) | tmp_bytes[5];
    
    return true;
  }
  
  return false;
}

// Read raw temperature values
// data     - input buffer to store value
// returns  - true on success
bool FC_IMU::temp_raw(int16_t *data) {
  uint8_t tmp_bytes[2];
  
  if (_tw.read_bytes(_i2c_address, TEMP_OUT_H, 2, tmp_bytes)) {
    data[0] = (( (uint16_t)(0 | tmp_bytes[0]) ) << 8) | tmp_bytes[1];
    
    return true;
  }
  
  return false;
}

ISR(TIMER2_OVF_vect) {
  sensor_update_int = true;
}
