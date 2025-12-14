#include "MKSServoE.h"

MKSServoE::MKSServoE(ICanBus& bus)
: _bus(bus), _targetId(0x01), _txId(0x01) {}

void MKSServoE::setTargetId(uint16_t id) { _targetId = id; }
void MKSServoE::setTxId(uint16_t id) { _txId = id; }

uint8_t MKSServoE::checksum(uint16_t id, const uint8_t* data, uint8_t len) {
  uint32_t sum = id;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return (uint8_t)(sum & 0xFF);
}

bool MKSServoE::enable() {
  uint8_t buf[2];
  buf[0] = 0xF3;
  buf[1] = checksum(_txId, buf, 1);
  CanFrame f{_txId, 2, {buf[0], buf[1],0,0,0,0,0,0}};
  return _bus.send(f);
}

bool MKSServoE::disable() {
  uint8_t buf[2];
  buf[0] = 0xF4;
  buf[1] = checksum(_txId, buf, 1);
  CanFrame f{_txId, 2, {buf[0], buf[1],0,0,0,0,0,0}};
  return _bus.send(f);
}

bool MKSServoE::queryBusStatus(uint8_t& status) {
  uint8_t buf[2];
  buf[0] = 0xF1;
  buf[1] = checksum(_txId, buf, 1);
  CanFrame f{_txId, 2, {buf[0], buf[1],0,0,0,0,0,0}};
  if (!_bus.send(f)) return false;
  if (!_bus.available()) return false;
  CanFrame rx;
  if (!_bus.read(rx)) return false;
  status = rx.data[1];
  return true;
}