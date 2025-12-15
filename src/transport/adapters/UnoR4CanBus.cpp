#include "UnoR4CanBus.h"

#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)

using arduino::CanMsg;

static bool toArduinoBitrate(uint32_t bps, CanBitRate &out) {
  switch (bps) {
    case 125000:  out = CanBitRate::BR_125k;  return true;
    case 250000:  out = CanBitRate::BR_250k;  return true;
    case 500000:  out = CanBitRate::BR_500k;  return true;
    case 1000000: out = CanBitRate::BR_1000k; return true;
    default: {
      return false;
    }
  }
}

bool UnoR4CanBus::begin(uint32_t bitrate) {
  CanBitRate br;
  if (!toArduinoBitrate(bitrate, br)) {
    return false;
  }
  return CAN.begin(br);
}

bool UnoR4CanBus::send(const CanFrame &frame) {
  CanMsg msg(arduino::CanStandardId(frame.id & 0x7FF), frame.dlc, frame.data);
  return CAN.write(msg);
}

bool UnoR4CanBus::available() {
  return CAN.available();
}

bool UnoR4CanBus::read(CanFrame &out) {
  if (CAN.available() == 0) {
    return false;
  }
  CanMsg msg = CAN.read();
  out.id = msg.getStandardId();
  out.dlc = msg.data_length;
  for (uint8_t i = 0; i < msg.data_length && i < 8; i++) {
    out.data[i] = msg.data[i];
  }
  for (uint8_t i = out.dlc; i < 8; i++) {
    out.data[i] = 0;
  }
  return true;
}

void UnoR4CanBus::setFilter(uint16_t, uint16_t) {
  // Filtering is not stable across Arduino_CAN versions on UNO R4; noop here.
}

#endif
