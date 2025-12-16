// Empty translation unit unless explicitly enabled for experimentation.
#if defined(MKSSERVOE_ENABLE_ADAPTER_TEMPLATE) && defined(PLACEHOLDER_BOARD_MACRO)

#include "AdapterTemplate.h"
// TODO: add backend includes here (e.g. <MCP2515.h>) once you enable this template.

// NOTE: AdapterTemplate.h currently keeps everything inline for easy copy/paste.
// If you want out-of-line definitions, remove the inline bodies in the header,
// then use this file to implement the methods under the same board guard.

// TODO: add a bitrate mapping helper here, mirroring UnoR4CanBus::toArduinoBitrate:
// static bool toBackendBitrate(uint32_t bps, BackendBitrate &out) {
//   switch (bps) {
//     case 125000:  out = BackendBitrate::BR_125k;  return true;
//     case 250000:  out = BackendBitrate::BR_250k;  return true;
//     case 500000:  out = BackendBitrate::BR_500k;  return true;
//     case 1000000: out = BackendBitrate::BR_1000k; return true;
//     default: return false;
//   }
// }

// TODO: translate between backend frame type and CanFrame, keeping these rules:
// - In send(), mask frame.id with 0x7FF before building the backend frame.
// - In read(), copy payload bytes up to dlc and zero out the rest (dlc..7).
// - If the backend can produce extended IDs, mask/reject and document the choice.

// Example implementation outline (remove the inline bodies from the header first):
// bool TemplateCanBusAdapter::begin(uint32_t bitrate) {
//   BackendBitrate br;
//   if (!toBackendBitrate(bitrate, br)) {
//     return false;
//   }
//   return backend.begin(br);
// }
//
// bool TemplateCanBusAdapter::send(const CanFrame &frame) {
//   uint16_t standardId = static_cast<uint16_t>(frame.id & 0x7FF);
//   // Build backend message with standardId, frame.dlc, frame.data.
//   return backend.send(msg);
// }
//
// bool TemplateCanBusAdapter::available() {
//   return backend.available();
// }
//
// bool TemplateCanBusAdapter::read(CanFrame &out) {
//   if (!backend.available()) {
//     return false;
//   }
//   auto msg = backend.read();
//   out.id = static_cast<uint16_t>(msg.id & 0x7FF);
//   out.dlc = msg.dlc;
//   for (uint8_t i = 0; i < out.dlc && i < 8; i++) {
//     out.data[i] = msg.data[i];
//   }
//   for (uint8_t i = out.dlc; i < 8; i++) {
//     out.data[i] = 0;
//   }
//   return true;
// }
//
// void TemplateCanBusAdapter::setFilter(uint16_t id, uint16_t mask) {
//   // Call backend filter API here; if unsupported/unstable, keep as a no-op and document it.
//   (void)id;
//   (void)mask;
// }

#endif // defined(MKSSERVOE_ENABLE_ADAPTER_TEMPLATE) && defined(PLACEHOLDER_BOARD_MACRO)
