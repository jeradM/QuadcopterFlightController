#ifndef __FC_WIRE_H__
#define __FC_WIRE_H__

#include "Arduino.h"

class FC_Wire {
public:
  FC_Wire();
  
  uint8_t read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t num_bytes, uint8_t *data);
  uint8_t read_byte(uint8_t dev_addr, uint8_t reg, uint8_t *data);
  uint8_t read_bits(uint8_t dev_addr, uint8_t reg, uint8_t start, uint8_t num_bits, uint8_t *data);
  uint8_t read_bit(uint8_t dev_addr, uint8_t reg, uint8_t bit);
  
  bool write_bytes(uint8_t dev_addr, uint8_t reg, uint8_t num_bytes, uint8_t *data);
  bool write_byte(uint8_t dev_addr, uint8_t reg, uint8_t data);
  bool write_bits(uint8_t dev_addr, uint8_t reg, uint8_t start, uint8_t length, uint8_t data);
  bool write_bit(uint8_t dev_addr, uint8_t reg, uint8_t bit, uint8_t data);
};

#endif