#include <Arduino.h>
#include "MKSServoE.h"
#include "protocol/MksPacking.h"

MKSServoE::MKSServoE(ICanBus& bus)
: _bus(bus), _targetId(0x01), _txId(0x01) {}

void MKSServoE::setTargetId(uint16_t id) { _targetId = id; _txId = id; }
void MKSServoE::setTxId(uint16_t id) { _txId = id; }

uint8_t MKSServoE::checksum(uint16_t id, const uint8_t* data, uint8_t len) const {
  uint32_t sum = id;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return (uint8_t)(sum & 0xFF);
}

bool MKSServoE::validateCrc(const CanFrame &frame) const {
  if (frame.dlc < 2) return false;
  uint8_t calc = checksum(frame.id, frame.data, frame.dlc - 1);
  return calc == frame.data[frame.dlc - 1];
}

bool MKSServoE::waitForResponse(uint8_t expectedCmd, CanFrame &rx, uint32_t timeoutMs) {
  unsigned long start = millis();
  while ((millis() - start) <= timeoutMs) {
    if (_bus.available()) {
      if (!_bus.read(rx)) continue;
      if (rx.id != _targetId) continue;
      if (rx.dlc == 0) continue;
      if (rx.data[0] != expectedCmd) continue;
      if (!validateCrc(rx)) continue;
      return true;
    }
    delay(1);
  }
  return false;
}

bool MKSServoE::sendCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t expectedRespCmd, CanFrame *response, uint32_t timeoutMs) {
  if (payloadLen > 6) return false; // cmd + payload + crc must fit into 8 bytes

  uint8_t buf[8] = {0};
  buf[0] = cmd;
  for (uint8_t i = 0; i < payloadLen; i++) buf[1 + i] = payload[i];
  const uint8_t crcIndex = 1 + payloadLen;
  buf[crcIndex] = checksum(_txId, buf, crcIndex);

  CanFrame tx{};
  tx.id = _txId;
  tx.dlc = crcIndex + 1;
  for (uint8_t i = 0; i < tx.dlc && i < 8; i++) tx.data[i] = buf[i];

  if (!_bus.send(tx)) return false;
  if (response) {
    return waitForResponse(expectedRespCmd, *response, timeoutMs);
  }
  return true;
}

bool MKSServoE::sendStatusCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t &statusOut, uint32_t timeoutMs, bool requireStatusSuccess) {
  CanFrame rx{};
  if (!sendCommand(cmd, payload, payloadLen, cmd, &rx, timeoutMs)) return false;
  if (rx.dlc < 3) return false;
  statusOut = rx.data[1];
  return requireStatusSuccess ? (statusOut == 1) : true;
}

bool MKSServoE::enableBus(bool enableState, uint8_t *statusOut, uint32_t timeoutMs) {
  uint8_t payload[1] = { static_cast<uint8_t>(enableState ? 1 : 0) };
  uint8_t status = 0;
  bool ok = sendStatusCommand(MKS::CMD_ENABLE_BUS, payload, 1, status, timeoutMs);
  if (statusOut) *statusOut = status;
  return ok;
}

bool MKSServoE::enable() {
  uint8_t status = 0;
  bool ok = enableBus(true, &status);
  return ok && status == 1;
}

bool MKSServoE::disable() {
  uint8_t status = 0;
  bool ok = enableBus(false, &status);
  return ok && status == 1;
}

bool MKSServoE::queryBusStatus(uint8_t& status, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_QUERY_STATUS, nullptr, 0, MKS::CMD_QUERY_STATUS, &rx, timeoutMs)) return false;
  if (rx.dlc < 3) return false;
  status = rx.data[1];
  return true;
}

bool MKSServoE::calibrateEncoder(uint8_t& status, uint32_t timeoutMs) {
  const uint8_t payload[1] = { 0x00 };
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_CALIBRATE_ENCODER, payload, 1, MKS::CMD_CALIBRATE_ENCODER, &rx, timeoutMs)) return false;
  if (rx.dlc < 3) return false;
  status = rx.data[1];
  return true; // status: 0=calibrating,1=success,2=fail
}

bool MKSServoE::writeUserId(uint32_t userId, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[4];
  MKS::put_u32_le(payload, userId);
  bool ok = sendStatusCommand(MKS::CMD_WRITE_USER_ID, payload, 4, status, timeoutMs);
  return ok;
}

bool MKSServoE::readUserId(uint32_t &userId, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_USER_ID, nullptr, 0, MKS::CMD_READ_USER_ID, &rx, timeoutMs)) return false;
  if (rx.dlc < 6) return false;
  userId = MKS::get_u32_le(&rx.data[1]);
  return true;
}

bool MKSServoE::readSpeedRpm(int16_t &rpm, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_SPEED_RPM, nullptr, 0, MKS::CMD_READ_SPEED_RPM, &rx, timeoutMs)) return false;
  if (rx.dlc < 4) return false;
  rpm = (int16_t)((rx.data[2] << 8) | rx.data[1]);
  return true;
}

bool MKSServoE::readEncoderAddition(int64_t &value, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_ENCODER_ADDITION, nullptr, 0, MKS::CMD_READ_ENCODER_ADDITION, &rx, timeoutMs)) return false;
  if (rx.dlc < 8) return false;
  int64_t v = 0;
  for (uint8_t i = 0; i < 6; i++) v |= ((int64_t)rx.data[1 + i]) << (8 * i);
  // sign extend from 48-bit
  if (rx.data[6] & 0x80) v |= ((int64_t)0xFFFF) << 48;
  value = v;
  return true;
}

bool MKSServoE::readEncoderCarry(int32_t &carry, uint16_t &value, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_ENCODER_CARRY, nullptr, 0, MKS::CMD_READ_ENCODER_CARRY, &rx, timeoutMs)) return false;
  if (rx.dlc < 8) return false;
  uint32_t c = MKS::get_u32_le(&rx.data[1]);
  carry = (int32_t)c;
  value = MKS::get_u16_le(&rx.data[5]);
  return true;
}

bool MKSServoE::setMode(uint8_t mode, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { mode };
  return sendStatusCommand(MKS::CMD_SET_MODE, payload, 1, status, timeoutMs);
}

bool MKSServoE::setCurrentMa(uint16_t ma, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[2];
  MKS::put_u16_le(payload, ma);
  return sendStatusCommand(MKS::CMD_SET_CURRENT_MA, payload, 2, status, timeoutMs);
}

bool MKSServoE::setMicrostep(uint8_t microstep, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { microstep };
  return sendStatusCommand(MKS::CMD_SET_MICROSTEP, payload, 1, status, timeoutMs);
}

bool MKSServoE::setDirection(uint8_t dir, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { (uint8_t)(dir ? 1 : 0) };
  return sendStatusCommand(MKS::CMD_SET_DIR, payload, 1, status, timeoutMs);
}

bool MKSServoE::setEnActive(uint8_t mode, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { mode };
  return sendStatusCommand(MKS::CMD_SET_EN_ACTIVE, payload, 1, status, timeoutMs);
}

bool MKSServoE::setPulseDelay(uint8_t delay, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { delay };
  return sendStatusCommand(MKS::CMD_SET_PULSE_DELAY, payload, 1, status, timeoutMs);
}

bool MKSServoE::runSpeed(uint8_t dir, uint16_t speedRpm, uint8_t acc, uint8_t &status, uint32_t timeoutMs) {
  if (speedRpm > 3000) speedRpm = 3000;
  uint8_t payload[3];
  payload[0] = (uint8_t)((dir ? 0x80 : 0x00) | ((speedRpm >> 8) & 0x7F));
  payload[1] = (uint8_t)(speedRpm & 0xFF);
  payload[2] = acc;
  return sendStatusCommand(MKS::CMD_SPEED_MODE, payload, 3, status, timeoutMs, /*requireStatusSuccess=*/false);
}

bool MKSServoE::setHomeConfig(uint8_t trigLevel, uint8_t homeDir, uint16_t homeSpeedRpm, uint8_t endLimitEnable, uint8_t mode, uint8_t &status, uint32_t timeoutMs) {
  if (homeSpeedRpm > 3000) homeSpeedRpm = 3000;
  uint8_t payload[6];
  payload[0] = trigLevel;
  payload[1] = homeDir;
  MKS::put_u16_le(&payload[2], homeSpeedRpm);
  payload[4] = endLimitEnable;
  payload[5] = mode;
  return sendStatusCommand(MKS::CMD_SET_HOME_PARAM, payload, 6, status, timeoutMs);
}

bool MKSServoE::goHome(uint8_t &status, uint32_t timeoutMs) {
  // status: 0 fail, 1 start, 2 success
  return sendStatusCommand(MKS::CMD_GO_HOME, nullptr, 0, status, timeoutMs, /*requireStatusSuccess=*/false);
}

bool MKSServoE::setAxisZero(uint8_t &status, uint32_t timeoutMs) {
  return sendStatusCommand(MKS::CMD_SET_AXIS_ZERO, nullptr, 0, status, timeoutMs);
}
