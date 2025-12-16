#include "AdapterTemplate.h"

/*
  This .cpp shows the recommended "real adapter" layout:
  - Keep the interface in the .h (class declaration).
  - Put the method bodies in the .cpp.

  Why have a .cpp at all?
  - It matches the pattern of UnoR4CanBus.
  - It keeps headers smaller and avoids accidental multiple-definition mistakes.
  - It is easier for beginners to see "what to change where".

  IMPORTANT:
  - This file is guarded by PLACEHOLDER_BOARD_MACRO.
  - That means the code below is NOT compiled in normal builds.
  - When you create a real adapter, you replace PLACEHOLDER_BOARD_MACRO with a real guard.

  ALSO IMPORTANT (most common beginner mistake):
  - Backend library includes must be inside the same guard.
    Otherwise users on other boards will get compile errors because the backend library isn't installed.
*/

#if defined(PLACEHOLDER_BOARD_MACRO)

// TODO: Put your backend library includes HERE, inside the guard.
// Example:
//   #include <mcp2515.h>
//   #include <SomeOtherCanDriver.h>

/*
  Bitrate mapping helper (mirrors UnoR4CanBus::toArduinoBitrate):

  Goal:
  - Convert generic bps (uint32_t) into whatever your backend expects.

  Pattern:
  - switch(bps)
  - set 'out'
  - return true
  - default: return false (unsupported bitrate)
*/
bool TemplateCanBusAdapter::toBackendBitrate(uint32_t bps, BackendBitrate &out) {
  switch (bps) {
    case 125000:  {      out = 125000;      return true;    }
    case 250000:  {      out = 250000;      return true;    }
    case 500000:  {      out = 500000;      return true;    }
    case 1000000: {      out = 1000000;     return true;    }
    default: {
      return false;
    }
  }
}

/*
  begin(bitrate):
  - Map bitrate -> backend bitrate type
  - Call backend.begin(...)
*/
bool TemplateCanBusAdapter::begin(uint32_t bitrate) {
  BackendBitrate br;
  if (!toBackendBitrate(bitrate, br)) {
    // Unsupported bitrate requested by the sketch.
    return false;
  }
  return _backend.begin(br);
}

/*
  send(frame):
  - Validate DLC
  - Mask ID to standard 11-bit
  - Convert CanFrame -> backend frame/message type
*/
bool TemplateCanBusAdapter::send(const CanFrame &frame) {
  // DLC must be 0..8.
  if (frame.dlc > 8) {
    return false;
  }

  // Standard 11-bit CAN ID (mask is important).
  const uint16_t standardId = static_cast<uint16_t>(frame.id & 0x7FF);

  // Build backend message.
  PlaceholderFrame msg{};
  msg.id = standardId;
  msg.dlc = frame.dlc;

  // Copy payload bytes.
  for (uint8_t i = 0; i < msg.dlc; i++) {
    msg.data[i] = frame.data[i];
  }

  // Zero padding bytes (avoid stale data).
  for (uint8_t i = msg.dlc; i < 8; i++) {
    msg.data[i] = 0;
  }

  return _backend.send(msg);
}

/*
  available():
  - Return whether there is at least one frame to read.
*/
bool TemplateCanBusAdapter::available() {
  return _backend.available();
}

/*
  read(out):
  - Return false if nothing available
  - Read one backend frame
  - Validate DLC
  - Fill CanFrame fields
  - Copy bytes + zero padding bytes
*/
bool TemplateCanBusAdapter::read(CanFrame &out) {
  if (!_backend.available()) {
    return false;
  }

  const PlaceholderFrame msg = _backend.read();

  // DLC must be 0..8.
  if (msg.dlc > 8) {
    return false;
  }

  // Standard 11-bit ID:
  // If your backend returns extended IDs and you want to reject them,
  // do that here. For now we mask to 11-bit like UNO R4 send() does.
  out.id = static_cast<uint16_t>(msg.id & 0x7FF);
  out.dlc = msg.dlc;

  // Copy payload bytes.
  for (uint8_t i = 0; i < out.dlc; i++) {
    out.data[i] = msg.data[i];
  }

  // Zero padding bytes.
  for (uint8_t i = out.dlc; i < 8; i++) {
    out.data[i] = 0;
  }

  return true;
}

/*
  setFilter(id, mask):
  - If your backend supports filters, call it here.
  - If not, keep it as a no-op and add a short comment (like UNO R4).
*/
void TemplateCanBusAdapter::setFilter(uint16_t id, uint16_t mask) {
  // If supported:
  // _backend.setFilter(id, mask);

  // If unsupported/unstable, keep no-op but accept parameters:
  (void)id;
  (void)mask;
}

#endif // defined(PLACEHOLDER_BOARD_MACRO)
