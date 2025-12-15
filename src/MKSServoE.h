#pragma once
#include <stdint.h>
#include "transport/ICanBus.h"
#include "command_map/MKSServoE_CAN_V1_0_1.h"

class MKSServoE {
public:
  explicit MKSServoE(ICanBus& bus);

  void setTargetId(uint16_t id);
  void setTxId(uint16_t id);

  struct VersionInfo {
    uint8_t series;
    uint8_t calibrationFlag;
    uint8_t hardwareVersion;
    uint8_t firmware[3];
  };

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
  bool runPositionMode1Relative(uint8_t dir, uint16_t speedRpm, uint8_t acc, int32_t pulses, uint8_t &status, uint32_t timeoutMs = 2000);
  bool runPositionMode2Absolute(uint8_t dir, uint16_t speedRpm, uint8_t acc, int32_t absPulses, uint8_t &status, uint32_t timeoutMs = 2000);
  bool runPositionMode3RelativeAxis(uint16_t speedRpm, uint8_t acc, int32_t relAxis, uint8_t &status, uint32_t timeoutMs = 2000);
  bool runPositionMode4AbsoluteAxis(uint16_t speedRpm, uint8_t acc, int32_t absAxis, uint8_t &status, uint32_t timeoutMs = 2000);
  bool emergencyStop(uint8_t &status, uint32_t timeoutMs = 50);

  bool setHomeConfig(uint8_t trigLevel, uint8_t homeDir, uint16_t homeSpeedRpm, uint8_t endLimitEnable, uint8_t mode, uint8_t &status, uint32_t timeoutMs = 50);
  bool goHome(uint8_t &status, uint32_t timeoutMs = 2000);
  bool setAxisZero(uint8_t &status, uint32_t timeoutMs = 50);
  bool setNoLimitHomeCurrent(uint16_t currentMa, uint8_t &status, uint32_t timeoutMs = 50);
  bool setNoLimitHomeParam(const uint8_t *payload, uint8_t payloadLen, uint8_t &status, uint32_t timeoutMs = 50);

  bool readInputPulses(int32_t &pulses, uint32_t timeoutMs = 50);
  bool readIoStatus(uint8_t &status, uint32_t timeoutMs = 50);
  bool readPositionError(int32_t &error, uint32_t timeoutMs = 50);
  bool readEnStatus(uint8_t &enable, uint32_t timeoutMs = 50);
  bool releaseStallProtection(uint8_t &status, uint32_t timeoutMs = 50);
  bool readStallState(uint8_t &status, uint32_t timeoutMs = 50);
  bool restoreDefaults(uint8_t &status, uint32_t timeoutMs = 200);
  bool readVersionInfo(VersionInfo &info, uint32_t timeoutMs = 50);
  bool restart(uint8_t &status, uint32_t timeoutMs = 200);
  bool readParam(uint8_t paramCode, uint8_t *dataOut, uint8_t maxLen, uint8_t &outLen, uint32_t timeoutMs = 50);

  bool writeIoPort(uint8_t almMask, uint8_t pendMask, uint8_t &status, uint32_t timeoutMs = 50);
  bool setStallProtectEnable(bool enable, uint8_t &status, uint32_t timeoutMs = 50);
  bool setStallTolerance(uint16_t tolerance, uint8_t &status, uint32_t timeoutMs = 50);
  bool setCanBitrate(uint8_t code, uint8_t &status, uint32_t timeoutMs = 50);
  bool setCanId(uint16_t id, uint8_t &status, uint32_t timeoutMs = 50);
  bool setRespondActive(uint8_t respond, uint8_t active, uint8_t &status, uint32_t timeoutMs = 50);
  bool setGroupId(uint16_t id, uint8_t &status, uint32_t timeoutMs = 50);
  bool lockAxis(bool enable, uint8_t &status, uint32_t timeoutMs = 50);
  bool remapLimitPort(uint8_t remap, uint8_t &status, uint32_t timeoutMs = 50);
  bool setPendDivOutput(const uint8_t *payload, uint8_t payloadLen, uint8_t &status, uint32_t timeoutMs = 50);
  bool saveCleanSpeedMode(bool save, uint8_t &status, uint32_t timeoutMs = 50);

private:
  ICanBus& _bus;
  uint16_t _targetId;
  uint16_t _txId;

  uint8_t checksum(const uint8_t* data, uint8_t len) const;
  bool waitForResponse(uint8_t expectedCmd, CanFrame &rx, uint32_t timeoutMs);
  bool validateCrc(const CanFrame &frame) const;
  bool sendStatusCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t &statusOut, uint32_t timeoutMs, bool requireStatusSuccess = true);
  bool sendCommand(uint8_t cmd, const uint8_t *payload, uint8_t payloadLen, uint8_t expectedRespCmd, CanFrame *response, uint32_t timeoutMs);
  bool packSpeedFields(uint8_t dir, uint16_t speedRpm, uint8_t acc, uint8_t *outBuf);
};
