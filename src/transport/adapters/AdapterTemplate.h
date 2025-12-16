#pragma once

#include "transport/ICanBus.h"

/*
  AdapterTemplate.{h,cpp}

  Purpose:
  - Help beginners add a new CAN backend adapter for this library.
  - This adapter must implement the ICanBus interface (begin/send/available/read/setFilter/poll).

  How to use this template:
  1) Copy these two files:
       AdapterTemplate.h  ->  <YourAdapter>.h
       AdapterTemplate.cpp -> <YourAdapter>.cpp
  2) Rename the class:
       TemplateCanBusAdapter -> <YourAdapter>
  3) Replace PLACEHOLDER_BOARD_MACRO with real selection guards.
     Examples of real guards:
       - Board-level: ARDUINO_UNOR4_MINIMA, ARDUINO_UNOR4_WIFI
       - Arch-level:  ARDUINO_ARCH_RENESAS_UNO
     You will also add a selection branch in AdapterSelector.h later.
  4) Add your backend library include(s) inside the guard in the .cpp.
     IMPORTANT: keep backend includes inside the guard so other boards don't need that library.

  Conventions used by this library (mirrors UnoR4CanBus):
  - Standard 11-bit CAN IDs:
      This library uses standard CAN IDs for commands (11-bit).
      In send(): always mask with (id & 0x7FF) before giving it to the backend.
      If your backend can produce extended IDs on read(), decide whether to reject or mask them.
  - DLC (data length) must be 0..8:
      Reject anything > 8 to avoid undefined behavior.
  - Read padding rule:
      After reading a frame, always fill out.data[dlc..7] with 0.
      This prevents "stale bytes" from previous frames.
  - Filters:
      Some backends have unstable filter APIs. If you can't support filters reliably,
      setFilter() may be a documented no-op (like UNO R4).
*/

/*
  This guard ensures the template code is not compiled in normal builds.

  If you are IMPLEMENTING a real adapter:
  - Replace PLACEHOLDER_BOARD_MACRO with your real guard, or remove the guard and put the guard
    around backend-specific includes/implementation in the .cpp.
*/
#if defined(PLACEHOLDER_BOARD_MACRO)

/*
  BackendBitrate:
  - Replace this with your backend's bitrate type (enum/int/etc).
  - UNO R4 uses CanBitRate enum; others may use integer bps or different enums.
*/
using BackendBitrate = uint32_t;

/*
  Placeholder types:
  - These exist only so the template can be read as a complete example.
  - Replace PlaceholderFrame with your backend frame type (or whatever the backend uses).
  - Replace PlaceholderBackend with your backend driver class / instance.
*/
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

  // Optional: filter API if your backend supports it.
  void setFilter(uint16_t, uint16_t) {}
};

/*
  The adapter class:
  - Must inherit ICanBus.
  - Must implement all virtual methods.
  - Keep the public API minimal: it is used by MKSServoE driver.
*/
class TemplateCanBusAdapter : public ICanBus {
public:
  /*
    begin(bitrate):
    - Called once in setup() by the sketch.
    - 'bitrate' is universal uint32_t in bps (125000/250000/500000/1000000 in our demos).
    - Your adapter maps it to backend-specific representation and calls backend begin/init.
  */
  bool begin(uint32_t bitrate) override;

  /*
    send(frame):
    - Sends one CAN frame.
    - Must use standard 11-bit ID: mask frame.id with 0x7FF.
    - Must respect DLC 0..8.
    - If backend requires a specific message object, translate here.
  */
  bool send(const CanFrame &frame) override;

  /*
    available():
    - Returns true if a frame can be read right now.
    - Keep it fast; it can be polled frequently.
  */
  bool available() override;

  /*
    read(out):
    - Reads one frame into 'out' if available; returns false if none available.
    - Must fill out.id and out.dlc.
    - Must copy payload bytes and then zero out out.data[out.dlc..7].
  */
  bool read(CanFrame &out) override;

  /*
    setFilter(id, mask):
    - Optional: configure receive filtering in the backend if supported.
    - If unsupported/unstable, keep as a no-op and document it.
  */
  void setFilter(uint16_t id, uint16_t mask) override;

  /*
    poll():
    - Some backends need a "pump" call to move data between IRQ/driver buffers.
    - UNO R4 doesn't need it, but the interface provides it for future backends.
    - Default: do nothing.
  */
  void poll() override {
    // No-op by default.
  }

private:
  /*
    Bitrate mapping helper:
    - Mirror UnoR4CanBus: switch-case mapping and return false on unsupported bps.
    - Keep it separate so begin() stays readable.
  */
  static bool toBackendBitrate(uint32_t bps, BackendBitrate &out);

  // The backend driver instance. Replace with your real backend object.
  PlaceholderBackend _backend;
};

#endif // defined(PLACEHOLDER_BOARD_MACRO)
