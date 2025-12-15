#include "UnoR4CanBus.h"

#if defined(ARDUINO_ARCH_RENESAS_UNO) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)

using arduino::CanMsg;

static CanBitRate toArduinoBitrate(uint32_t bps) {
  switch (bps) {
    case 125000:  return CanBitRate::BR_125k;
    case 250000:  return CanBitRate::BR_250k;
    case 500000:  return CanBitRate::BR_500k;
    case 1000000: return CanBitRate::BR_1000k;
    default:      return CanBitRate::BR_500k;
  }
}

bool UnoR4CanBus::begin(uint32_t bitrate) {
  return CAN.begin(toArduinoBitrate(bitrate));
}

bool UnoR4CanBus::send(const CanFrame &frame) {
  CanMsg msg(frame.id, frame.dlc, frame.data);
  return CAN.write(msg) == 1;
}

bool UnoR4CanBus::available() {
  return CAN.available() > 0;
}

bool UnoR4CanBus::read(CanFrame &out) {
  if (CAN.available() == 0) return false;
  CanMsg msg = CAN.read();
  out.id = msg.id;
  out.dlc = msg.data_length;
  for (uint8_t i = 0; i < msg.data_length && i < 8; i++) out.data[i] = msg.data[i];
  return true;
}

void UnoR4CanBus::setFilter(uint16_t id, uint16_t mask) {
  CAN.setFilterMask_Standard(mask);
  CAN.setFilterId_Standard(0, id);
}

#else
bool UnoR4CanBus::begin(uint32_t) { return false; }
bool UnoR4CanBus::send(const CanFrame&) { return false; }
bool UnoR4CanBus::available() { return false; }
bool UnoR4CanBus::read(CanFrame&) { return false; }
void UnoR4CanBus::setFilter(uint16_t, uint16_t) {}
#endif
