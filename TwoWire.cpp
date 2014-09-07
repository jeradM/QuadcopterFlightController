#include "TwoWire.h"

// Read Multiple Bytes from Slave Device
// dev_addr  - I2C address of target device
// reg       - Register address where reading should begin
// num_bytes - Number of bytes to attempt to read
// data      - Pointer to Buffer to store read data
// returns   - the number of bytes read
uint8_t TwoWire::read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t num_bytes, uint8_t *data) {
  uint8_t cnt = 0;
  
  Wire.beginTransmission(dev_addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(dev_addr);
  Wire.requestFrom(dev_addr, num_bytes);
  
  while (Wire.available()) {
    data[cnt++] = Wire.read();
  }
  
  return cnt;
}

// Read a single byte from a slave device
// dev_addr  - I2C address of the target device
// reg       - Register address to read
// data      - Pointer to buffer for read data
// returns   - 1 for success - 0 means read failed
uint8_t TwoWire::read_byte(uint8_t dev_addr, uint8_t reg, uint8_t *data) {
  return read_bytes(dev_addr, reg, 1, data);
}

// Read multiple individual bits from slave device
// dev_addr  - I2C address of target device
// reg       - Register address from which to read bits
// start     - Starting bit position -- 0 (LSB) - 7 (MSB)
// num_bits  - Number of consecutive bits to read from 'start' (counting down)
//              e.g. start: 7 - num_bits: 3 -> reads bits 7:5
// data      - Reference to buffer for read data
// returns   - bit values (from length -> bit 0)
//             e.g. bits 6:4 of 01100101 returns 00000110
uint8_t TwoWire::read_bits(uint8_t dev_addr, uint8_t reg, uint8_t start, uint8_t num_bits, uint8_t *data) {
  if (start > 7 || num_bits > 7) return -1;
  
  uint8_t tmp_byte = 0;
  uint8_t shift = (start - length) + 1;
  
  uint8_t bit_mask = ((1 << length) - 1) << shift;
  
  if (read_byte(dev_addr, reg, &tmp_byte)) {
    data = (tmp_byte & bit_mask) >> shift;
    return 1;
  }
  
  return 0;
}

// Read a single bit from slave device
// dev_addr  - I2C address of target device
// reg       - Register address from which to read bit
// bit       - Position of interesting bit
// returns   - bit value (1 or 0)
uint8_t TwoWire::read_bit(uint8_t dev_addr, uint8_t reg, uint8_t bit) {
  uint8_t b;
  read_bits(dev_addr, reg, bit, 1, &b);
  return b;
}

// Write multiple bytes to slave device
// dev_addr   - I2C address of slave device
// reg        - Register address at which to start writing
// num_bytes  - Number of bytes to write
// data       - Pointer to output buffer holding data to write
// returns    - true if write succeeded
bool TwoWire::write_bytes(uint8_t dev_addr, uint8_t reg, uint8_t num_bytes, uint8_t *data) {
  Wire.beginTransmission(dev_addr);
  Wire.write((uint8_t) reg);
  
  for (int i = 0; i < num_bytes; i++) {
    Wire.write((uint8_t) data[i]);
  }
  
  return (Wire.endTransmission() == 0); // 0 means success
}

// Write single byte to slave device
// dev_addr   - I2C address of slave device
// reg        - Register address to write to
// data       - Byte to write
// returns    - true if write succeeded
bool TwoWire::write_byte(uint8_t dev_addr, uint8_t reg, uint8_t data) {
  return write_bytes(dev_addr, reg, 1, &data);
}

// Write multiple individual bits to slave device
// dev_addr   - I2C address of slave device
// reg        - Register address to write to
// start      - Starting bit position
// length     - number of consecutive bits to write
// data       - bits to write (Shifted to LSBs)
// returns    - true if write succeeded
bool TwoWire::write_bits(uint8_t dev_addr, uint8_t reg, uint8_t start, uint8_t length, uint8_t data) {
  uint8_t b;
  
  if (read_byte(dev_addr, reg, &b) == 1) {
    data <<= ((start - length) + 1);
    b &= data;
    return write_byte(dev_addr, reg, b);
  }
  
  return false;
}

// Write individual bit to slave device
// dev_addr   - I2C address of slave device
// reg        - Register address to write to
// bit        - Position of interesting bit
// data       - 1 or 0
// returns    - true if write succeeded
bool TwoWire::write_bit(uint8_t dev_addr, uint8_t reg, uint8_t bit, uint8_t data) {
  uint8_t b;
  
  if (data > 1) return false;
  
  if (read_byte(dev_addr, reg, &b) == 1) {
    b &= (data << bit);
    return write_byte(dev_addr, reg, b);
  }
  
  return false;
}
