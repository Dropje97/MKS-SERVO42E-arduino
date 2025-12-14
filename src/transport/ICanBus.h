#pragma once
#include <stdint.h>

struct CanFrame {
  uint16_t id;
  uint8_t  dlc;
  uint8_t  data[8];
};

class ICanBus {
public:
  virtual bool begin(uint32_t bitrate) = 0;
  virtual bool send(const CanFrame& f) = 0;
  virtual bool available() = 0;
  virtual bool read(CanFrame& out) = 0;
  virtual void setFilter(uint16_t id, uint16_t mask) = 0;
  virtual ~ICanBus() = default;
};