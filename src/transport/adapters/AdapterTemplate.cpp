// Empty translation unit unless explicitly enabled for experimentation.
#if defined(PLACEHOLDER_BOARD_MACRO)

#include "AdapterTemplate.h"
// TODO: add backend includes here (e.g. <MCP2515.h>) once you enable this template.

// NOTE: AdapterTemplate.h currently keeps everything inline for easy copy/paste.
// If you want out-of-line definitions, remove the inline bodies in the header,
// then use this file to implement the methods under the same board guard.

// Placeholder backend types so this file stays buildable when the guard is set.
// Replace these with the real backend message type and driver instance.
namespace {
struct PlaceholderFrame {
  uint16_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8] = {0};
};

class PlaceholderBackend {
public:
  bool begin(BackendBitrate) { return false; }
  bool send(const PlaceholderFrame &) { return false; }
  bool available() const { return false; }
  PlaceholderFrame read() const { return PlaceholderFrame(); }
} backend;
} // namespace

// Bitrate mapping helper, mirroring UnoR4CanBus::toArduinoBitrate.
static bool toBackendBitrate(uint32_t bps, BackendBitrate &out) {
  switch (bps) {
    case 125000:  out = 125000;  return true;
    case 250000:  out = 250000;  return true;
    case 500000:  out = 500000;  return true;
    case 1000000: out = 1000000; return true;
    default: {
      return false;
    }
  }
}

// Translate between backend frame type and CanFrame, keeping these rules:
// - In send(), mask frame.id with 0x7FF before building the backend frame.
// - In read(), copy payload bytes up to dlc and zero out the rest (dlc..7).
// - If the backend can produce extended IDs, mask/reject and document the choice.

// Example implementation outline (remove the inline bodies from the header first):
bool TemplateCanBusAdapter::begin(uint32_t bitrate) {
  BackendBitrate br;
  if (!toBackendBitrate(bitrate, br)) {
    return false;
  }
  return backend.begin(br);
}

bool TemplateCanBusAdapter::send(const CanFrame &frame) {
  uint16_t standardId = static_cast<uint16_t>(frame.id & 0x7FF);
  // Build backend message with standardId, frame.dlc, frame.data.
  PlaceholderFrame msg;
  msg.id = standardId;
  msg.dlc = frame.dlc;
  for (uint8_t i = 0; i < msg.dlc && i < 8; i++) {
    msg.data[i] = frame.data[i];
  }
  for (uint8_t i = msg.dlc; i < 8; i++) {
    msg.data[i] = 0;
  }
  return backend.send(msg);
}

bool TemplateCanBusAdapter::available() {
  return backend.available();
}

bool TemplateCanBusAdapter::read(CanFrame &out) {
  if (!backend.available()) {
    return false;
  }
  auto msg = backend.read();
  out.id = static_cast<uint16_t>(msg.id & 0x7FF);
  out.dlc = msg.dlc;
  for (uint8_t i = 0; i < out.dlc && i < 8; i++) {
    out.data[i] = msg.data[i];
  }
  for (uint8_t i = out.dlc; i < 8; i++) {
    out.data[i] = 0;
  }
  return true;
}

void TemplateCanBusAdapter::setFilter(uint16_t id, uint16_t mask) {
  // Call backend filter API here; if unsupported/unstable, keep as a no-op and document it.
  (void)id;
  (void)mask;
}

#endif // defined(PLACEHOLDER_BOARD_MACRO)
