\
#include "UnoR4CanBus.h"

#if defined(ARDUINO_ARCH_RENESAS_UNO) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)

static CanBitRate toArduinoBitrate(uint32_t bps) {
  switch (bps) {
    case 125000:  return CanBitRate::BR_125k;
    case 250000:  return CanBitRate::BR_250k;
    case 500000:  return CanBitRate::BR_500k;
    case 1000000: return CanBitRate::BR_1M;
    default:      return CanBitRate::BR_500k;
  }
}

bool UnoR4CanBus::begin(uint32_t bitrate) {
  return CAN.begin(toArduinoBitrate(bitrate));
}

bool UnoR4CanBus::write(const CanFrame &frame) {
  CanMsg msg(frame.id, frame.dlc, frame.data);
  return CAN.write(msg);
}

bool UnoR4CanBus::available() {
  return CAN.available();
}

bool UnoR4CanBus::read(CanFrame &out) {
  CanMsg msg;
  if (!CAN.read(msg)) return false;
  out.id = msg.id;
  out.dlc = msg.data_length;
  for (uint8_t i = 0; i < out.dlc && i < 8; i++) out.data[i] = msg.data[i];
  return true;
}

#else
bool UnoR4CanBus::begin(uint32_t) { return false; }
bool UnoR4CanBus::write(const CanFrame&) { return false; }
bool UnoR4CanBus::available() { return false; }
bool UnoR4CanBus::read(CanFrame&) { return false; }
#endif
