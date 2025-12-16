#pragma once
#include "transport/ICanBus.h"

// Copy/paste template for a new CAN adapter.
// Rename TemplateCanBusAdapter to match the hardware you wrap (e.g. Mcp2515CanBus).
// Keep everything header-only while drafting; you can later split into .h/.cpp.
//
// Checklist (matches UnoR4CanBus conventions):
// 1) Include the backend CAN library inside the board guard (e.g. <MCP2515.h>).
// 2) Bitrate mapping: add a helper that converts bps to the backend enum/const.
//    - Use a switch/lookup like UnoR4CanBus::toArduinoBitrate.
//    - Return false on unsupported rates so the caller can handle it.
// 3) CAN IDs: this codebase uses standard 11-bit IDs.
//    - In send(), mask with (frame.id & 0x7FF) before passing to the backend.
//    - If the backend returns extended IDs, mask or reject them and document it.
// 4) Read padding: after copying bytes [0..dlc-1], zero out out.data[dlc..7]
//    just like UnoR4CanBus::read.
// 5) Filters: implement setFilter() if the backend supports it. If the API is
//    unstable or unsupported, keep it as a documented no-op (see UNO R4 comment).
// 6) Add board guards around includes and implementation so it only builds
//    where valid, then extend AdapterSelector.h with an #elif branch.
// 7) Avoid dynamic allocation and keep calls non-blocking.

// Replace this guard with the real board/library check for your adapter.
#if defined(PLACEHOLDER_BOARD_MACRO)

// Backend-specific bitrate type placeholder; replace with your library's enum.
using BackendBitrate = uint32_t;

// Map from bps to backend bitrate; return false for unsupported values.
static bool toBackendBitrate(uint32_t bps, BackendBitrate &out) {
  switch (bps) {
    case 125000:  out = bps; return true; // Replace with backend enum.
    case 250000:  out = bps; return true;
    case 500000:  out = bps; return true;
    case 1000000: out = bps; return true;
    default: {
      return false;
    }
  }
}

class TemplateCanBusAdapter : public ICanBus {
public:
  // Initialize the CAN controller at the requested bitrate.
  bool begin(uint32_t bitrate) override {
    BackendBitrate br;
    if (!toBackendBitrate(bitrate, br)) {
      return false;
    }
    (void)br; // TODO: call backend begin/br mapping here.
    return false;
  }

  // Send a single CAN frame (11-bit IDs). Mask to 0x7FF before passing on.
  bool send(const CanFrame &frame) override {
    uint16_t standardId = static_cast<uint16_t>(frame.id & 0x7FF);
    (void)standardId; // TODO: build backend frame and send it.
    (void)frame;
    return false;
  }

  // Non-blocking check for pending frames.
  bool available() override {
    // TODO: replace with backend available/readiness check.
    return false;
  }

  // Read the next frame into 'out'. Return true only when a frame was read.
  bool read(CanFrame &out) override {
    // TODO: pull a frame from the backend and populate out.id/out.dlc/out.data.
    // If the backend delivers extended IDs, mask them to 11-bit or reject.
    out.id = 0;
    out.dlc = 0;
    // Copy payload bytes: for (uint8_t i = 0; i < out.dlc && i < 8; i++) { out.data[i] = backend.data[i]; }
    for (uint8_t i = out.dlc; i < 8; i++) {
      out.data[i] = 0;
    }
    return false;
  }

  // Configure acceptance filters; document if the backend does not support it.
  void setFilter(uint16_t id, uint16_t mask) override {
    (void)id;
    (void)mask;
    // TODO: apply backend filter setup; keep as no-op if unsupported/unstable.
  }
};

#endif // defined(PLACEHOLDER_BOARD_MACRO)
