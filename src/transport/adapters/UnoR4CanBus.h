#pragma once
#include "../ICanBus.h"

#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
#include <Arduino_CAN.h>

class UnoR4CanBus : public ICanBus {
public:
  bool begin(uint32_t bitrate) override;
  bool send(const CanFrame &frame) override;
  bool available() override;
  bool read(CanFrame &out) override;
  void setFilter(uint16_t id, uint16_t mask) override; // no-op on UNO R4
};

#else
#error "UnoR4CanBus requires Arduino UNO R4"
#endif
