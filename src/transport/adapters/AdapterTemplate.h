#pragma once
#include "../ICanBus.h"

// Copy/paste template for a new CAN adapter.
// Rename TemplateCanBusAdapter to match the hardware you wrap (e.g. Mcp2515CanBus).
// Keep everything in a header or split into .h/.cpp as needed; this header is standalone.
//
// Checklist for implementing a new adapter:
// 1) Decide which Arduino CAN library you wrap and include it here.
//    - Example: #include <MCP2515.h> or the vendor-specific header.
// 2) Map bitrate values expected by this library to your hardware enum/const.
//    - Accept the same bitrate units (bps) as the rest of the codebase.
//    - Provide a lookup similar to UnoR4CanBus::toArduinoBitrate to reject unsupported rates.
// 3) Implement begin/send/available/read/setFilter to satisfy ICanBus.
//    - begin(uint32_t bitrate): configure the hardware, return false on failure.
//    - send(const CanFrame&): send a standard 11-bit frame using the provided id/dlc/data.
//    - available(): return true when a frame can be read without blocking.
//    - read(CanFrame& out): fill id, dlc, data[0..7]; zero any unused bytes so callers are safe.
//    - setFilter(uint16_t id, uint16_t mask): configure acceptance filters; if unsupported, leave a
//      no-op but document why.
// 4) Handle DLC and padding:
//    - Respect the dlc provided by the caller and ensure the underlying library sees the same length.
//    - If the hardware/library pads frames, explicitly clear unused bytes (see UnoR4CanBus).
// 5) Add board guards:
//    - Wrap the implementation with #if defined(BOARD_MACRO) checks so it only builds where valid.
//    - Place the same guard around any includes that rely on board-specific libraries.
// 6) Update AdapterSelector.h:
//    - Add an #elif branch that includes your new header and sets using CanBusAdapter = YourType;
//    - Keep the error message for unsupported boards untouched.
// 7) Keep everything non-blocking and avoid dynamic allocation.

// Replace this guard with the real board/library check for your adapter.
#if defined(PLACEHOLDER_BOARD_MACRO)

class TemplateCanBusAdapter : public ICanBus {
public:
  // Initialize the CAN controller at the requested bitrate.
  bool begin(uint32_t bitrate) override {
    (void)bitrate; // Map to your library-specific enum/values here.
    return false;  // Replace with the library call result.
  }

  // Send a single CAN frame. Return false if the hardware cannot accept it.
  bool send(const CanFrame &frame) override {
    (void)frame; // Translate CanFrame to your library's message type.
    return false;
  }

  // Non-blocking check for pending frames.
  bool available() override {
    return false; // Replace with the library's available/readiness check.
  }

  // Read the next frame into 'out'. Return true only when a frame was read.
  bool read(CanFrame &out) override {
    (void)out; // Copy id/dlc/data from the library's message into 'out'.
    return false;
  }

  // Configure acceptance filters; no-op if the hardware does not support it.
  void setFilter(uint16_t id, uint16_t mask) override {
    (void)id;
    (void)mask;
  }
};

#endif // defined(PLACEHOLDER_BOARD_MACRO)
