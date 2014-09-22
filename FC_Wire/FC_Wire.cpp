#include <Wire.h>
#include "FC_Wire.h"

FC_Wire::FC_Wire() {
  Wire.begin();
}

// Read Multiple Bytes from Slave Device
// dev_addr  - I2C address of target device
// reg       - Register address where reading should begin
// length    - Number of bytes to attempt to read
// data      - Pointer to Buffer to store read data
// returns   - the number of bytes read
uint8_t FC_Wire::read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t length, uint8_t *data) {
  uint8_t cnt = 0;
  
  Wire.beginTransmission(dev_addr);
  Wire.write(reg);
  Wire.endTransmission();
  //Wire.beginTransmission(dev_addr);
  Wire.requestFrom(dev_addr, length);
  
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
uint8_t FC_Wire::read_byte(uint8_t dev_addr, uint8_t reg, uint8_t *data) {
  return read_bytes(dev_addr, reg, 1, data);
}

// Read 16-bit Packets from Slave Device
// dev_addr  - I2C address of target device
// reg       - Register address where reading should begin
// length    - Number of packets to attempt to read
// data      - Pointer to Buffer to store read data
// returns   - the number of packets read
uint8_t FC_Wire::read_packets(uint8_t dev_addr, uint8_t reg, uint8_t length, uint16_t *data) {
  uint8_t tmp_data[64];
  uint8_t bytes_read = read_bytes(dev_addr, reg, (length * 2), tmp_data);
  if (bytes_read == (length * 2)) {
    for (int i = 0; i < length; i++) {
      data[i] = (((uint16_t)tmp_data[i * 2]) << 8) | tmp_data[(i * 2) + 1];
    }
    return bytes_read / 2;
  }
  
  return 0;
}

// Read multiple individual bits from slave device
// dev_addr  - I2C address of target device
// reg       - Register address from which to read bits
// start     - Starting bit position -- 0 (LSB) - 7 (MSB)
// length    - Number of consecutive bits to read from 'start' (counting down)
//              e.g. start: 7 - length: 3 -> reads bits 7:5
// data      - Reference to buffer for read data
// returns   - bit values (from length -> bit 0)
//             e.g. bits 6:4 of 01100101 returns 00000110
uint8_t FC_Wire::read_bits(uint8_t dev_addr, uint8_t reg, uint8_t start, uint8_t length, uint8_t *data) {
  if (start > 7 || length > 7) return -1;
  
  uint8_t tmp_byte = 0;
  uint8_t shift = (start - length) + 1;
  
  uint8_t bit_mask = ((1 << length) - 1) << shift;
  
  if (read_byte(dev_addr, reg, &tmp_byte)) {
    data[0] = 0 | (tmp_byte & bit_mask) >> shift;
    return 1;
  }
  
  return 0;
}

// Read a single bit from slave device
// dev_addr  - I2C address of target device
// reg       - Register address from which to read bit
// bit       - Position of interesting bit
// returns   - bit value (1 or 0)
uint8_t FC_Wire::read_bit(uint8_t dev_addr, uint8_t reg, uint8_t bit) {
  uint8_t b;
  read_bits(dev_addr, reg, bit, 1, &b);
  return b;
}

// Write multiple bytes to slave device
// dev_addr   - I2C address of slave device
// reg        - Register address at which to start writing
// length  - Number of bytes to write
// data       - Pointer to output buffer holding data to write
// returns    - true if write succeeded
bool FC_Wire::write_bytes(uint8_t dev_addr, uint8_t reg, uint8_t length, uint8_t *data) {
  Wire.beginTransmission(dev_addr);
  Wire.write((uint8_t) reg);
  
  for (int i = 0; i < length; i++) {
    Wire.write(data[i]);
  }
  
  return (Wire.endTransmission() == 0); // 0 means success
}

// Write single byte to slave device
// dev_addr   - I2C address of slave device
// reg        - Register address to write to
// data       - Byte to write
// returns    - true if write succeeded
bool FC_Wire::write_byte(uint8_t dev_addr, uint8_t reg, uint8_t data) {
  return write_bytes(dev_addr, reg, 1, &data);
}

// Write multiple individual bits to slave device
// dev_addr   - I2C address of slave device
// reg        - Register address to write to
// start      - Starting bit position
// length     - number of consecutive bits to write
// data       - bits to write (Shifted to LSBs)
// returns    - true if write succeeded
bool FC_Wire::write_bits(uint8_t dev_addr, uint8_t reg, uint8_t start, uint8_t length, uint8_t data) {
  uint8_t b;
  
  if (read_byte(dev_addr, reg, &b) == 1) {
    uint8_t bitmask = ((1 << length) - 1) << (start + 1 - length);
    data <<= (start + 1 - length);
    data &= bitmask;
    b &= ~(bitmask);
    b |= data;
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
bool FC_Wire::write_bit(uint8_t dev_addr, uint8_t reg, uint8_t bit, uint8_t data) {
  uint8_t b;
  
  if (data > 1) return false;
  
  if (read_byte(dev_addr, reg, &b) == 1) {
    if (data == 0) {
      b &= ~(1 << bit);
    }
    else {
      b |= (1 << bit);
    }
    return write_byte(dev_addr, reg, b);
  }
  
  return false;
}
