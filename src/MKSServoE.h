#pragma once
#include <stdint.h>
#include "transport/ICanBus.h"

class MKSServoE {
public:
  explicit MKSServoE(ICanBus& bus);

  void setTargetId(uint16_t id);
  void setTxId(uint16_t id);

  bool enable();
  bool disable();
  bool queryBusStatus(uint8_t& status);

private:
  ICanBus& _bus;
  uint16_t _targetId;
  uint16_t _txId;

  uint8_t checksum(uint16_t id, const uint8_t* data, uint8_t len);
};