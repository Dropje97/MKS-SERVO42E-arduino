#pragma once
#include <stdint.h>
#include "transport/ICanBus.h"
#include "command_map/MKSServoE_CAN_V1_0_1.h"

class MKSServoE {
public:
  enum ERROR : uint8_t {
    ERROR_OK = 0,
    ERROR_BUS_SEND,
    ERROR_TIMEOUT,
    ERROR_BAD_FRAME,
    ERROR_BAD_CRC,
    ERROR_BAD_RESPONSE,
    ERROR_INVALID_ARG,
    ERROR_DEVICE_STATUS_FAIL
  };

  explicit MKSServoE(ICanBus& bus);

  void setTargetId(uint16_t id);
  void setTxId(uint16_t id);

  struct VersionInfo {
    uint8_t series;
    uint8_t calibrationFlag;
    uint8_t hardwareVersion;
    uint8_t firmware[3];
  };

  ERROR enable();
  ERROR disable();
  ERROR enableBus(bool enableState, uint8_t *statusOut = nullptr, uint32_t timeoutMs = 50);
  ERROR queryBusStatus(uint8_t& status, uint32_t timeoutMs = 50);

  ERROR calibrateEncoder(uint8_t& status, uint32_t timeoutMs = 1000);

  ERROR writeUserId(uint32_t userId, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR readUserId(uint32_t &userId, uint32_t timeoutMs = 50);

  ERROR readSpeedRpm(int16_t &rpm, uint32_t timeoutMs = 50);
  ERROR readEncoderAddition(int64_t &value, uint32_t timeoutMs = 50);
  ERROR readEncoderCarry(int32_t &carry, uint16_t &value, uint32_t timeoutMs = 50);

  ERROR setMode(uint8_t mode, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setCurrentMa(uint16_t ma, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setMicrostep(uint8_t microstep, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setDirection(uint8_t dir, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setEnActive(uint8_t mode, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setPulseDelay(uint8_t delay, uint8_t &status, uint32_t timeoutMs = 50);

  ERROR runSpeed(uint8_t dir, uint16_t speedRpm, uint8_t acc, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR runPositionMode1Relative(uint8_t dir, uint16_t speedRpm, uint8_t acc, int32_t pulses, uint8_t &status, uint32_t timeoutMs = 2000);
  ERROR runPositionMode2Absolute(uint8_t dir, uint16_t speedRpm, uint8_t acc, int32_t absPulses, uint8_t &status, uint32_t timeoutMs = 2000);
  ERROR runPositionMode3RelativeAxis(uint16_t speedRpm, uint8_t acc, int32_t relAxis, uint8_t &status, uint32_t timeoutMs = 2000);
  ERROR runPositionMode4AbsoluteAxis(uint16_t speedRpm, uint8_t acc, int32_t absAxis, uint8_t &status, uint32_t timeoutMs = 2000);
  ERROR emergencyStop(uint8_t &status, uint32_t timeoutMs = 50);

  ERROR setHomeConfig(uint8_t trigLevel, uint8_t homeDir, uint16_t homeSpeedRpm, uint8_t endLimitEnable, uint8_t mode, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR goHome(uint8_t &status, uint32_t timeoutMs = 2000);
  ERROR setAxisZero(uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setNoLimitHomeCurrent(uint16_t currentMa, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setNoLimitHomeParam(const uint8_t *payload, uint8_t payloadLen, uint8_t &status, uint32_t timeoutMs = 50);

  ERROR readInputPulses(int32_t &pulses, uint32_t timeoutMs = 50);
  ERROR readIoStatus(uint8_t &status, uint32_t timeoutMs = 50);
  ERROR readPositionError(int32_t &error, uint32_t timeoutMs = 50);
  ERROR readEnStatus(uint8_t &enable, uint32_t timeoutMs = 50);
  ERROR releaseStallProtection(uint8_t &status, uint32_t timeoutMs = 50);
  ERROR readStallState(uint8_t &status, uint32_t timeoutMs = 50);
  ERROR restoreDefaults(uint8_t &status, uint32_t timeoutMs = 200);
  ERROR readVersionInfo(VersionInfo &info, uint32_t timeoutMs = 50);
  ERROR restart(uint8_t &status, uint32_t timeoutMs = 200);
  ERROR readParam(uint8_t paramCode, uint8_t *dataOut, uint8_t maxLen, uint8_t &outLen, uint32_t timeoutMs = 50);

  ERROR writeIoPort(uint8_t almMask, uint8_t pendMask, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setStallProtectEnable(bool enable, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setStallTolerance(uint16_t tolerance, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setCanBitrate(uint8_t code, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setCanId(uint16_t id, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setRespondActive(uint8_t respond, uint8_t active, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setGroupId(uint16_t id, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR lockAxis(bool enable, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR remapLimitPort(uint8_t remap, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR setPendDivOutput(const uint8_t *payload, uint8_t payloadLen, uint8_t &status, uint32_t timeoutMs = 50);
  ERROR saveCleanSpeedMode(bool save, uint8_t &status, uint32_t timeoutMs = 50);

private:
  ICanBus& _bus;
  uint16_t _targetId;
  uint16_t _txId;

  uint8_t checksum(const uint8_t* data, uint8_t len) const;
  bool validateCrc(const CanFrame &frame) const;
  ERROR waitForResponse(uint8_t expectedCmd, CanFrame &rx, uint32_t timeoutMs);
  ERROR sendStatusCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t &statusOut, uint32_t timeoutMs, bool requireStatusSuccess = true);
  ERROR sendCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t expectedRespCmd, CanFrame *response, uint32_t timeoutMs);
  void packSpeedFields(uint8_t dir, uint16_t speedRpm, uint8_t acc, uint8_t *outBuf);
};
