#pragma once
#include <stddef.h>
#include "ICanBus.h"

// Buffered wrapper that keeps a small queue of received CAN frames.
// It delegates send/filter to the underlying adapter while letting callers
// drain already-read frames without blocking on hardware.
template <typename Adapter, size_t Capacity = 4>
class BufferedCanBus : public ICanBus {
public:
  BufferedCanBus() : _bus(), _head(0), _tail(0), _count(0) {}

  bool begin(uint32_t bitrate) override {
    return _bus.begin(bitrate);
  }

  bool send(const CanFrame &f) override {
    return _bus.send(f);
  }

  void setFilter(uint16_t id, uint16_t mask) override {
    _bus.setFilter(id, mask);
  }

  Adapter& underlying() {
    return _bus;
  }

private:
  void poll() {
    while (_count < Capacity && _bus.available()) {
      CanFrame f{};
      if (!_bus.read(f)) {
        break;
      }
      _buffer[_head] = f;
      _head = (_head + 1) % Capacity;
      _count++;
    }
  }

  Adapter _bus;
  CanFrame _buffer[Capacity];
  size_t _head;
  size_t _tail;
  size_t _count;

  bool available() override {
    poll();
    return _count > 0;
  }

  bool read(CanFrame &out) override {
    poll();
    if (_count == 0) {
      return false;
    }
    out = _buffer[_tail];
    _tail = (_tail + 1) % Capacity;
    _count--;
    return true;
  }

  friend class MKSServoE;
};
