#include "UnoR4CanBus.h"

#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)

namespace {
bool toArduinoBitrate(uint32_t bps, CanBitRate &out) {
  switch (bps) {
    case 125000:  { out = CanBitRate::BR_125k;  return true; }
    case 250000:  { out = CanBitRate::BR_250k;  return true; }
    case 500000:  { out = CanBitRate::BR_500k;  return true; }
    case 1000000: { out = CanBitRate::BR_1000k; return true; }
    default: {
      return false;
    }
  }
}
} // namespace

bool UnoR4CanBus::begin(uint32_t bitrate) {
  CanBitRate mapped;
  if (!toArduinoBitrate(bitrate, mapped)) {
    return false;
  }
  return CAN.begin(mapped);
}

bool UnoR4CanBus::send(const CanFrame &frame) {
  if (frame.dlc > 8) {
    return false;
  }
  uint8_t data[8] = {0};
  for (uint8_t i = 0; i < frame.dlc; i++) {
    data[i] = frame.data[i];
  }
  CanMsg msg(CanStandardId(frame.id), frame.dlc, data);
  return CAN.write(msg) > 0;
}

bool UnoR4CanBus::available() {
  return CAN.available() > 0;
}

bool UnoR4CanBus::read(CanFrame &out) {
  if (CAN.available() == 0) {
    return false;
  }
  CanMsg msg = CAN.read();
  if (msg.data_length > 8) {
    return false;
  }
  out.id = static_cast<uint16_t>(msg.getStandardId());
  out.dlc = msg.data_length;
  for (uint8_t i = 0; i < out.dlc; i++) {
    out.data[i] = msg.data[i];
  }
  for (uint8_t i = out.dlc; i < 8; i++) {
    out.data[i] = 0;
  }
  return true;
}

void UnoR4CanBus::setFilter(uint16_t id, uint16_t mask) {
  // UNO R4 CAN filter API is not exposed in Arduino_CAN; parameters are unused.
  (void)id;
  (void)mask;
}

#endif // defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
