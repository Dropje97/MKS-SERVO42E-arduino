#pragma once
#include <stdint.h>
#include "transport/ICanBus.h"
#include "command_map/MKSServoE_CAN_V1_0_1.h"

class MKSServoE {
public:
  explicit MKSServoE(ICanBus& bus);

  void setTargetId(uint16_t id);
  void setTxId(uint16_t id);

  bool enable();
  bool disable();
  bool enableBus(bool enableState, uint8_t *statusOut = nullptr, uint32_t timeoutMs = 50);
  bool queryBusStatus(uint8_t& status, uint32_t timeoutMs = 50);

  bool calibrateEncoder(uint8_t& status, uint32_t timeoutMs = 1000);

  bool writeUserId(uint32_t userId, uint8_t &status, uint32_t timeoutMs = 50);
  bool readUserId(uint32_t &userId, uint32_t timeoutMs = 50);

  bool readSpeedRpm(int16_t &rpm, uint32_t timeoutMs = 50);
  bool readEncoderAddition(int64_t &value, uint32_t timeoutMs = 50);
  bool readEncoderCarry(int32_t &carry, uint16_t &value, uint32_t timeoutMs = 50);

  bool setMode(uint8_t mode, uint8_t &status, uint32_t timeoutMs = 50);
  bool setCurrentMa(uint16_t ma, uint8_t &status, uint32_t timeoutMs = 50);
  bool setMicrostep(uint8_t microstep, uint8_t &status, uint32_t timeoutMs = 50);
  bool setDirection(uint8_t dir, uint8_t &status, uint32_t timeoutMs = 50);
  bool setEnActive(uint8_t mode, uint8_t &status, uint32_t timeoutMs = 50);
  bool setPulseDelay(uint8_t delay, uint8_t &status, uint32_t timeoutMs = 50);

  bool runSpeed(uint8_t dir, uint16_t speedRpm, uint8_t acc, uint8_t &status, uint32_t timeoutMs = 50);

  bool setHomeConfig(uint8_t trigLevel, uint8_t homeDir, uint16_t homeSpeedRpm, uint8_t endLimitEnable, uint8_t mode, uint8_t &status, uint32_t timeoutMs = 50);
  bool goHome(uint8_t &status, uint32_t timeoutMs = 2000);
  bool setAxisZero(uint8_t &status, uint32_t timeoutMs = 50);

private:
  ICanBus& _bus;
  uint16_t _targetId;
  uint16_t _txId;

  uint8_t checksum(uint16_t id, const uint8_t* data, uint8_t len) const;
  bool waitForResponse(uint8_t expectedCmd, CanFrame &rx, uint32_t timeoutMs);
  bool validateCrc(const CanFrame &frame) const;
  bool sendStatusCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t &statusOut, uint32_t timeoutMs, bool requireStatusSuccess = true);
  bool sendCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t expectedRespCmd, CanFrame *response, uint32_t timeoutMs);
};
