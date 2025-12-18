#include <Arduino.h>
#include "MKSServoE.h"
#include "protocol/MksPacking.h"
#include "protocol/MksCrc.h"

MKSServoE::MKSServoE(ICanBus& bus)
: _bus(bus), _targetId(0x01), _txId(0x01), _slots(), _reservedCount{0}, _nextSequence(0) {}

void MKSServoE::setTargetId(uint16_t id) { _targetId = id; }
void MKSServoE::setTxId(uint16_t id) { _txId = id; }

uint8_t MKSServoE::checksum(const uint8_t* data, uint8_t len) const {
  return MKS::crc8_sum_plus1(data, len);
}

bool MKSServoE::validateCrc(const CanFrame &frame) const {
  if (frame.dlc < 2) {
    return false;
  }
  uint8_t calc = checksum(frame.data, frame.dlc - 1);
  return calc == frame.data[frame.dlc - 1];
}

void MKSServoE::clearSlot(uint8_t slotIndex) {
  if (slotIndex >= RESPONSE_QUEUE_SLOTS) {
    return;
  }
  ResponseSlot &slot = _slots[slotIndex];
  slot.used = false;
  slot.cmd = 0;
  slot.head = 0;
  slot.count = 0;
  for (uint8_t i = 0; i < RESPONSE_QUEUE_DEPTH; i++) {
    slot.sequence[i] = 0;
  }
}

int8_t MKSServoE::findSlot(uint8_t cmd) const {
  for (uint8_t i = 0; i < RESPONSE_QUEUE_SLOTS; i++) {
    if (_slots[i].used && _slots[i].cmd == cmd) {
      return (int8_t)i;
    }
  }
  return -1;
}

bool MKSServoE::isReserved(uint8_t cmd) const {
  return _reservedCount[cmd] > 0;
}

void MKSServoE::reserve(uint8_t cmd) {
  if (_reservedCount[cmd] < 255) {
    _reservedCount[cmd]++;
  }
}

void MKSServoE::unreserve(uint8_t cmd) {
  if (_reservedCount[cmd] > 0) {
    _reservedCount[cmd]--;
  }
}

int8_t MKSServoE::allocateSlot(uint8_t cmd) {
  int8_t existing = findSlot(cmd);
  if (existing >= 0) {
    return existing;
  }

  for (uint8_t i = 0; i < RESPONSE_QUEUE_SLOTS; i++) {
    if (!_slots[i].used) {
      _slots[i].used = true;
      _slots[i].cmd = cmd;
      _slots[i].head = 0;
      _slots[i].count = 0;
      return (int8_t)i;
    }
  }

  int8_t candidate = -1;
  uint32_t bestSeq = 0xFFFFFFFFu;
  for (uint8_t i = 0; i < RESPONSE_QUEUE_SLOTS; i++) {
    if (!_slots[i].used || _slots[i].count == 0) {
      continue;
    }
    if (isReserved(_slots[i].cmd)) {
      continue;
    }
    uint32_t seq = _slots[i].sequence[_slots[i].head];
    if (seq < bestSeq) {
      bestSeq = seq;
      candidate = (int8_t)i;
    }
  }

  if (candidate == -1) {
    for (uint8_t i = 0; i < RESPONSE_QUEUE_SLOTS; i++) {
      if (!_slots[i].used || _slots[i].count == 0) {
        continue;
      }
      uint32_t seq = _slots[i].sequence[_slots[i].head];
      if (seq < bestSeq) {
        bestSeq = seq;
        candidate = (int8_t)i;
      }
    }
  }

  if (candidate >= 0) {
    clearSlot((uint8_t)candidate);
    ResponseSlot &slot = _slots[candidate];
    slot.used = true;
    slot.cmd = cmd;
    slot.head = 0;
    slot.count = 0;
  }
  return candidate;
}

void MKSServoE::enqueueFrame(uint8_t slotIndex, const CanFrame &frame) {
  if (slotIndex >= RESPONSE_QUEUE_SLOTS) {
    return;
  }
  ResponseSlot &slot = _slots[slotIndex];
  if (slot.count == RESPONSE_QUEUE_DEPTH) {
    slot.head = (uint8_t)((slot.head + 1) % RESPONSE_QUEUE_DEPTH);
    slot.count--;
  }
  uint8_t tail = (uint8_t)((slot.head + slot.count) % RESPONSE_QUEUE_DEPTH);
  slot.frames[tail] = frame;
  slot.sequence[tail] = _nextSequence++;
  slot.count++;
  slot.used = true;
  slot.cmd = frame.data[0];
}

bool MKSServoE::popFrame(uint8_t slotIndex, CanFrame &outFrame) {
  if (slotIndex >= RESPONSE_QUEUE_SLOTS) {
    return false;
  }
  ResponseSlot &slot = _slots[slotIndex];
  if (!slot.used || slot.count == 0) {
    return false;
  }
  outFrame = slot.frames[slot.head];
  slot.head = (uint8_t)((slot.head + 1) % RESPONSE_QUEUE_DEPTH);
  slot.count--;
  if (slot.count == 0) {
    clearSlot(slotIndex);
  }
  return true;
}

void MKSServoE::poll(uint8_t maxFrames) {
  uint8_t handled = 0;
  while (handled < maxFrames && _bus.available()) {
    CanFrame rx{};
    if (!_bus.read(rx)) {
      break;
    }
    handled++;
    if (rx.dlc < 2) {
      continue;
    }
    if (rx.id != _targetId) {
      continue;
    }
    if (!validateCrc(rx)) {
      continue;
    }
    int8_t slotIndex = allocateSlot(rx.data[0]);
    if (slotIndex >= 0) {
      enqueueFrame((uint8_t)slotIndex, rx);
    }
  }
}

MKSServoE::ERROR MKSServoE::pollResponse(uint8_t expectedCmd, CanFrame &rx) {
  poll(DEFAULT_MAX_FRAMES);
  int8_t slotIndex = findSlot(expectedCmd);
  if (slotIndex < 0) {
    return ERROR_NO_RESPONSE_AVAILABLE;
  }
  if (!popFrame((uint8_t)slotIndex, rx)) {
    return ERROR_NO_RESPONSE_AVAILABLE;
  }
  unreserve(expectedCmd);
  return ERROR_OK;
}

MKSServoE::ERROR MKSServoE::pollAnyResponse(uint8_t &cmdOut, CanFrame &rx, bool skipReserved) {
  poll(DEFAULT_MAX_FRAMES);
  int8_t candidate = -1;
  uint32_t bestSeq = 0xFFFFFFFFu;
  for (uint8_t i = 0; i < RESPONSE_QUEUE_SLOTS; i++) {
    if (!_slots[i].used || _slots[i].count == 0) {
      continue;
    }
    uint8_t cmd = _slots[i].cmd;
    if (skipReserved && isReserved(cmd)) {
      continue;
    }
    uint32_t seq = _slots[i].sequence[_slots[i].head];
    if (candidate == -1 || seq < bestSeq) {
      bestSeq = seq;
      candidate = (int8_t)i;
    }
  }
  if (candidate < 0) {
    return ERROR_NO_RESPONSE_AVAILABLE;
  }
  cmdOut = _slots[candidate].cmd;
  if (!popFrame((uint8_t)candidate, rx)) {
    return ERROR_NO_RESPONSE_AVAILABLE;
  }
  unreserve(cmdOut);
  return ERROR_OK;
}

MKSServoE::ERROR MKSServoE::waitForResponse(uint8_t expectedCmd, CanFrame &rx, uint32_t timeoutMs) {
  reserve(expectedCmd);
  const uint32_t start = millis();
  while ((uint32_t)(millis() - start) <= timeoutMs) {
    poll(DEFAULT_MAX_FRAMES);
    int8_t slotIndex = findSlot(expectedCmd);
    if (slotIndex >= 0 && popFrame((uint8_t)slotIndex, rx)) {
      unreserve(expectedCmd);
      return ERROR_OK;
    }
  }
  unreserve(expectedCmd);
  return ERROR_TIMEOUT;
}

MKSServoE::ERROR MKSServoE::sendCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t expectedRespCmd, CanFrame *response, uint32_t timeoutMs) {
  if (payloadLen > 6) {
    return ERROR_INVALID_ARG;
  }

  uint8_t buf[8] = {0};
  buf[0] = cmd;
  for (uint8_t i = 0; i < payloadLen; i++) {
    buf[1 + i] = payload[i];
  }
  const uint8_t crcIndex = 1 + payloadLen;
  buf[crcIndex] = checksum(buf, crcIndex);

  CanFrame tx{};
  tx.id = _txId;
  tx.dlc = crcIndex + 1;
  for (uint8_t i = 0; i < tx.dlc && i < 8; i++) {
    tx.data[i] = buf[i];
  }

  if (!_bus.send(tx)) {
    return ERROR_BUS_SEND;
  }
  if (response) {
    return waitForResponse(expectedRespCmd, *response, timeoutMs);
  }
  return ERROR_OK;
}

MKSServoE::ERROR MKSServoE::sendStatusCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t &statusOut, uint32_t timeoutMs, bool requireStatusSuccess, bool waitForResponse) {
  if (!waitForResponse) {
    statusOut = 0;
    reserve(cmd);
    MKSServoE::ERROR asyncRc = sendCommand(cmd, payload, payloadLen, cmd, nullptr, timeoutMs);
    if (asyncRc != ERROR_OK) {
      unreserve(cmd);
    }
    return asyncRc;
  }

  CanFrame rx{};
  MKSServoE::ERROR rc = sendCommand(cmd, payload, payloadLen, cmd, &rx, timeoutMs);
  if (rc != ERROR_OK) {
    return rc;
  }
  if (rx.dlc < 3) {
    return ERROR_BAD_FRAME;
  }
  statusOut = rx.data[1];
  if (requireStatusSuccess && statusOut == 0) {
    return ERROR_DEVICE_STATUS_FAIL;
  }
  return ERROR_OK;
}
