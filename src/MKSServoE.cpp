#include <Arduino.h>
#include "MKSServoE.h"
#include "protocol/MksPacking.h"
#include "protocol/MksCrc.h"

MKSServoE::MKSServoE(ICanBus& bus)
: _bus(bus), _targetId(0x01), _txId(0x01) {}

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

bool MKSServoE::waitForResponse(uint8_t expectedCmd, CanFrame &rx, uint32_t timeoutMs) {
  unsigned long start = millis();
  while ((millis() - start) <= timeoutMs) {
    if (_bus.available()) {
      if (!_bus.read(rx)) {
        continue;
      }
      if (rx.id != _targetId) {
        continue;
      }
      if (rx.dlc == 0) {
        continue;
      }
      if (rx.data[0] != expectedCmd) {
        continue;
      }
      if (!validateCrc(rx)) {
        continue;
      }
      return true;
    }
    delay(1);
  }
  return false;
}

bool MKSServoE::sendCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t expectedRespCmd, CanFrame *response, uint32_t timeoutMs) {
  if (payloadLen > 6) {
    return false; // cmd + payload + crc must fit into 8 bytes
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
    return false;
  }
  if (response) {
    return waitForResponse(expectedRespCmd, *response, timeoutMs);
  }
  return true;
}

bool MKSServoE::sendStatusCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t &statusOut, uint32_t timeoutMs, bool requireStatusSuccess) {
  CanFrame rx{};
  if (!sendCommand(cmd, payload, payloadLen, cmd, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 3) {
    return false;
  }
  statusOut = rx.data[1];
  return requireStatusSuccess ? (statusOut == 1) : true;
}

bool MKSServoE::packSpeedFields(uint8_t dir, uint16_t speedRpm, uint8_t acc, uint8_t *outBuf) {
  uint16_t clamped = speedRpm;
  if (clamped > 3000) {
    clamped = 3000;
  }
  outBuf[0] = (uint8_t)((dir ? 0x80 : 0x00) | ((clamped >> 8) & 0x7F));
  outBuf[1] = (uint8_t)(clamped & 0xFF);
  outBuf[2] = acc;
  return true;
}

bool MKSServoE::enableBus(bool enableState, uint8_t *statusOut, uint32_t timeoutMs) {
  uint8_t payload[1] = { static_cast<uint8_t>(enableState ? 1 : 0) };
  uint8_t status = 0;
  bool ok = sendStatusCommand(MKS::CMD_ENABLE_BUS, payload, 1, status, timeoutMs);
  if (statusOut) {
    *statusOut = status;
  }
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
  if (!sendCommand(MKS::CMD_QUERY_STATUS, nullptr, 0, MKS::CMD_QUERY_STATUS, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 3) {
    return false;
  }
  status = rx.data[1];
  return true;
}

bool MKSServoE::calibrateEncoder(uint8_t& status, uint32_t timeoutMs) {
  const uint8_t payload[1] = { 0x00 };
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_CALIBRATE_ENCODER, payload, 1, MKS::CMD_CALIBRATE_ENCODER, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 3) {
    return false;
  }
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
  if (!sendCommand(MKS::CMD_READ_USER_ID, nullptr, 0, MKS::CMD_READ_USER_ID, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 6) {
    return false;
  }
  userId = MKS::get_u32_le(&rx.data[1]);
  return true;
}

bool MKSServoE::readSpeedRpm(int16_t &rpm, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_SPEED_RPM, nullptr, 0, MKS::CMD_READ_SPEED_RPM, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 4) {
    return false;
  }
  rpm = (int16_t)((rx.data[2] << 8) | rx.data[1]);
  return true;
}

bool MKSServoE::readEncoderAddition(int64_t &value, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_ENCODER_ADDITION, nullptr, 0, MKS::CMD_READ_ENCODER_ADDITION, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 8) {
    return false;
  }
  int64_t v = 0;
  for (uint8_t i = 0; i < 6; i++) {
    v |= ((int64_t)rx.data[1 + i]) << (8 * i);
  }
  if (rx.data[6] & 0x80) {
    v |= ((int64_t)0xFFFF) << 48;
  }
  value = v;
  return true;
}

bool MKSServoE::readEncoderCarry(int32_t &carry, uint16_t &value, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_ENCODER_CARRY, nullptr, 0, MKS::CMD_READ_ENCODER_CARRY, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 8) {
    return false;
  }
  uint32_t c = MKS::get_u32_le(&rx.data[1]);
  carry = (int32_t)c;
  value = MKS::get_u16_le(&rx.data[5]);
  return true;
}

bool MKSServoE::readInputPulses(int32_t &pulses, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_INPUT_PULSES, nullptr, 0, MKS::CMD_READ_INPUT_PULSES, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 6) {
    return false;
  }
  uint32_t raw = MKS::get_u32_le(&rx.data[1]);
  pulses = (int32_t)raw;
  return true;
}

bool MKSServoE::readIoStatus(uint8_t &status, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_IO_STATUS, nullptr, 0, MKS::CMD_READ_IO_STATUS, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 3) {
    return false;
  }
  status = rx.data[1];
  return true;
}

bool MKSServoE::readPositionError(int32_t &error, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_POS_ERROR, nullptr, 0, MKS::CMD_READ_POS_ERROR, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 6) {
    return false;
  }
  uint32_t raw = MKS::get_u32_le(&rx.data[1]);
  error = (int32_t)raw;
  return true;
}

bool MKSServoE::readEnStatus(uint8_t &enable, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_EN_STATUS, nullptr, 0, MKS::CMD_READ_EN_STATUS, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 3) {
    return false;
  }
  enable = rx.data[1];
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
  uint8_t payload[3];
  packSpeedFields(dir, speedRpm, acc, payload);
  return sendStatusCommand(MKS::CMD_SPEED_MODE, payload, 3, status, timeoutMs, /*requireStatusSuccess=*/false);
}

static void putAxis(uint8_t *buf, int32_t value) {
  MKS::put_i24_le(buf, value);
}

bool MKSServoE::runPositionMode1Relative(uint8_t dir, uint16_t speedRpm, uint8_t acc, int32_t pulses, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[6];
  packSpeedFields(dir, speedRpm, acc, payload);
  putAxis(&payload[3], pulses);
  return sendStatusCommand(MKS::CMD_POS_MODE1_REL_PULSES, payload, 6, status, timeoutMs, /*requireStatusSuccess=*/false);
}

bool MKSServoE::runPositionMode2Absolute(uint8_t dir, uint16_t speedRpm, uint8_t acc, int32_t absPulses, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[6];
  packSpeedFields(dir, speedRpm, acc, payload);
  putAxis(&payload[3], absPulses);
  return sendStatusCommand(MKS::CMD_POS_MODE2_ABS_PULSES, payload, 6, status, timeoutMs, /*requireStatusSuccess=*/false);
}

bool MKSServoE::runPositionMode3RelativeAxis(uint16_t speedRpm, uint8_t acc, int32_t relAxis, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[6];
  packSpeedFields(0, speedRpm, acc, payload);
  putAxis(&payload[3], relAxis);
  return sendStatusCommand(MKS::CMD_POS_MODE3_REL_AXIS, payload, 6, status, timeoutMs, /*requireStatusSuccess=*/false);
}

bool MKSServoE::runPositionMode4AbsoluteAxis(uint16_t speedRpm, uint8_t acc, int32_t absAxis, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[6];
  packSpeedFields(0, speedRpm, acc, payload);
  putAxis(&payload[3], absAxis);
  return sendStatusCommand(MKS::CMD_POS_MODE4_ABS_AXIS, payload, 6, status, timeoutMs, /*requireStatusSuccess=*/false);
}

bool MKSServoE::emergencyStop(uint8_t &status, uint32_t timeoutMs) {
  return sendStatusCommand(MKS::CMD_EMERGENCY_STOP, nullptr, 0, status, timeoutMs, /*requireStatusSuccess=*/false);
}

bool MKSServoE::setHomeConfig(uint8_t trigLevel, uint8_t homeDir, uint16_t homeSpeedRpm, uint8_t endLimitEnable, uint8_t mode, uint8_t &status, uint32_t timeoutMs) {
  uint16_t clamped = homeSpeedRpm;
  if (clamped > 3000) {
    clamped = 3000;
  }
  uint8_t payload[6];
  payload[0] = trigLevel;
  payload[1] = homeDir;
  MKS::put_u16_le(&payload[2], clamped);
  payload[4] = endLimitEnable;
  payload[5] = mode;
  return sendStatusCommand(MKS::CMD_SET_HOME_PARAM, payload, 6, status, timeoutMs);
}

bool MKSServoE::goHome(uint8_t &status, uint32_t timeoutMs) {
  return sendStatusCommand(MKS::CMD_GO_HOME, nullptr, 0, status, timeoutMs, /*requireStatusSuccess=*/false);
}

bool MKSServoE::setAxisZero(uint8_t &status, uint32_t timeoutMs) {
  return sendStatusCommand(MKS::CMD_SET_AXIS_ZERO, nullptr, 0, status, timeoutMs);
}

bool MKSServoE::setNoLimitHomeCurrent(uint16_t currentMa, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[2];
  MKS::put_u16_le(payload, currentMa);
  return sendStatusCommand(MKS::CMD_SET_NOLIMIT_HOME_CURRENT, payload, 2, status, timeoutMs);
}

bool MKSServoE::setNoLimitHomeParam(const uint8_t *payload, uint8_t payloadLen, uint8_t &status, uint32_t timeoutMs) {
  if (payloadLen > 6) {
    return false;
  }
  return sendStatusCommand(MKS::CMD_SET_NOLIMIT_HOME_PARAM, payload, payloadLen, status, timeoutMs);
}

bool MKSServoE::releaseStallProtection(uint8_t &status, uint32_t timeoutMs) {
  return sendStatusCommand(MKS::CMD_RELEASE_STALL_PROTECT, nullptr, 0, status, timeoutMs);
}

bool MKSServoE::readStallState(uint8_t &status, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_STALL_STATE, nullptr, 0, MKS::CMD_READ_STALL_STATE, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 3) {
    return false;
  }
  status = rx.data[1];
  return true;
}

bool MKSServoE::setStallProtectEnable(bool enable, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { static_cast<uint8_t>(enable ? 1 : 0) };
  return sendStatusCommand(MKS::CMD_SET_STALL_PROTECT_ENABLE, payload, 1, status, timeoutMs);
}

bool MKSServoE::setStallTolerance(uint16_t tolerance, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[2];
  MKS::put_u16_le(payload, tolerance);
  return sendStatusCommand(MKS::CMD_SET_STALL_TOLERANCE, payload, 2, status, timeoutMs);
}

bool MKSServoE::setCanBitrate(uint8_t code, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { code };
  return sendStatusCommand(MKS::CMD_SET_CAN_BITRATE, payload, 1, status, timeoutMs);
}

bool MKSServoE::setCanId(uint16_t id, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[2];
  MKS::put_u16_le(payload, id);
  return sendStatusCommand(MKS::CMD_SET_CAN_ID, payload, 2, status, timeoutMs);
}

bool MKSServoE::setRespondActive(uint8_t respond, uint8_t active, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[2] = { respond, active };
  return sendStatusCommand(MKS::CMD_SET_RESPOND_ACTIVE, payload, 2, status, timeoutMs);
}

bool MKSServoE::setGroupId(uint16_t id, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[2];
  MKS::put_u16_le(payload, id);
  return sendStatusCommand(MKS::CMD_SET_GROUP_ID, payload, 2, status, timeoutMs);
}

bool MKSServoE::lockAxis(bool enable, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { static_cast<uint8_t>(enable ? 1 : 0) };
  return sendStatusCommand(MKS::CMD_LOCK_AXIS, payload, 1, status, timeoutMs);
}

bool MKSServoE::remapLimitPort(uint8_t remap, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { remap };
  return sendStatusCommand(MKS::CMD_REMAP_LIMIT_PORT, payload, 1, status, timeoutMs);
}

bool MKSServoE::setPendDivOutput(const uint8_t *payload, uint8_t payloadLen, uint8_t &status, uint32_t timeoutMs) {
  if (payloadLen > 6) {
    return false;
  }
  return sendStatusCommand(MKS::CMD_PEND_DIV_OUTPUT, payload, payloadLen, status, timeoutMs);
}

bool MKSServoE::writeIoPort(uint8_t almMask, uint8_t pendMask, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[2] = { almMask, pendMask };
  return sendStatusCommand(MKS::CMD_WRITE_IO_PORT, payload, 2, status, timeoutMs);
}

bool MKSServoE::restoreDefaults(uint8_t &status, uint32_t timeoutMs) {
  return sendStatusCommand(MKS::CMD_RESTORE_DEFAULTS, nullptr, 0, status, timeoutMs);
}

bool MKSServoE::readVersionInfo(VersionInfo &info, uint32_t timeoutMs) {
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_VERSION_INFO, nullptr, 0, MKS::CMD_READ_VERSION_INFO, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 6) {
    return false;
  }
  info.series = rx.data[1];
  info.calibrationFlag = rx.data[2];
  info.hardwareVersion = rx.data[3];
  info.firmware[0] = rx.data[4];
  info.firmware[1] = rx.data[5];
  info.firmware[2] = (rx.dlc > 6) ? rx.data[6] : 0;
  return true;
}

bool MKSServoE::restart(uint8_t &status, uint32_t timeoutMs) {
  return sendStatusCommand(MKS::CMD_RESTART, nullptr, 0, status, timeoutMs);
}

bool MKSServoE::readParam(uint8_t paramCode, uint8_t *dataOut, uint8_t maxLen, uint8_t &outLen, uint32_t timeoutMs) {
  uint8_t payload[1] = { paramCode };
  CanFrame rx{};
  if (!sendCommand(MKS::CMD_READ_PARAM, payload, 1, MKS::CMD_READ_PARAM, &rx, timeoutMs)) {
    return false;
  }
  if (rx.dlc < 3) {
    return false;
  }
  if (rx.data[1] != paramCode) {
    return false;
  }
  uint8_t available = (uint8_t)(rx.dlc - 3);
  outLen = available;
  uint8_t copyLen = available;
  if (copyLen > maxLen) {
    copyLen = maxLen;
  }
  for (uint8_t i = 0; i < copyLen; i++) {
    dataOut[i] = rx.data[2 + i];
  }
  return true;
}

bool MKSServoE::saveCleanSpeedMode(bool save, uint8_t &status, uint32_t timeoutMs) {
  uint8_t payload[1] = { static_cast<uint8_t>(save ? 0xC8 : 0xCA) };
  return sendStatusCommand(MKS::CMD_SAVE_CLEAN_SPEEDMODE, payload, 1, status, timeoutMs);
}
