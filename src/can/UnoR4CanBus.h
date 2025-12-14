\
#pragma once
#include "ICanBus.h"

#if defined(ARDUINO_ARCH_RENESAS_UNO) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  #include <Arduino_CAN.h>
#endif

class UnoR4CanBus : public ICanBus {
public:
  bool begin(uint32_t bitrate) override;
  bool write(const CanFrame &frame) override;
  bool available() override;
  bool read(CanFrame &out) override;
};
