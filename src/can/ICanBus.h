\
#pragma once
#include <stdint.h>
#include <stddef.h>

struct CanFrame {
  uint32_t id = 0;   // 11-bit standard ID
  uint8_t  dlc = 0;
  uint8_t  data[8] = {0};
};

class ICanBus {
public:
  virtual ~ICanBus() = default;

  virtual bool begin(uint32_t bitrate) = 0;          // bitrate in bits/s (125000, 250000, 500000, 1000000)
  virtual bool write(const CanFrame &frame) = 0;
  virtual bool available() = 0;
  virtual bool read(CanFrame &out) = 0;

  // Optional: for adapters that need polling/dispatching
  virtual void poll() {}
};
