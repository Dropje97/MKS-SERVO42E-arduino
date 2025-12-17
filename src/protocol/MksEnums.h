#pragma once
#include <stdint.h>

/*
  MksEnums.h

  Enum values + bit masks extracted from:
  "MKS SERVO42E/57E ONLY_CAN User Manual V1.0.1"

  Notes:
  - This header intentionally contains only small enums/bitmasks for discrete values.
  - Numeric ranges (speed, current, axis, etc.) are NOT enumerated here.
  - Where the manual is ambiguous or inconsistent, comments call it out and the raw
    numeric values from the manual are preserved.
*/

namespace MKS {

  // ---------------------------------------------------------------------------
  // Generic status used by many "set/config" commands: status=1 success, status=0 fail
  // ---------------------------------------------------------------------------
  enum class AckStatus : uint8_t {
    Fail    = 0x00,
    Success = 0x01,
  };

  // ---------------------------------------------------------------------------
  // Work mode (CMD_SET_MODE / 0x82, byte2 "mode")
  //
  // Manual uses "mode = X0 / X1 / 02 / 03 / X4 / 05" where:
  //   X = 0  -> with encoder
  //   X = 1  -> without encoder
  // and the low nibble is the mode number.
  // ---------------------------------------------------------------------------
  enum class WorkMode : uint8_t {
    // Open-loop pulse/pulse
    PulsePulse_OpenLoop_WithEncoder    = 0x00, // X0
    PulsePulse_OpenLoop_WithoutEncoder = 0x10, // X0 with X=1

    // Open-loop pulse/dir
    PulseDir_OpenLoop_WithEncoder      = 0x01, // X1
    PulseDir_OpenLoop_WithoutEncoder   = 0x11, // X1 with X=1

    // Closed-loop pulse/pulse
    PulsePulse_ClosedLoop              = 0x02,

    // Closed-loop pulse/dir
    PulseDir_ClosedLoop                = 0x03,

    // CAN open-loop
    Can_OpenLoop_WithEncoder           = 0x04, // X4
    Can_OpenLoop_WithoutEncoder        = 0x14, // X4 with X=1

    // CAN closed-loop
    Can_ClosedLoop                     = 0x05,
  };

  // ---------------------------------------------------------------------------
  // Microstep / subdivisions (CMD_SET_MICROSTEP / 0x84, byte2 "microstep")
  //
  // Manual lists these supported values:
  //   0, 2, 4, 8, 16, 32, 64, 128, 5, 10, 20, 25, 40, 50, 100, 200
  // with note: "0 means 256 subdivisions".
  // ---------------------------------------------------------------------------
  enum class Microstep : uint8_t {
    MS_256 = 0,
    MS_2   = 2,
    MS_4   = 4,
    MS_8   = 8,
    MS_16  = 16,
    MS_32  = 32,
    MS_64  = 64,
    MS_128 = 128,
    MS_5   = 5,
    MS_10  = 10,
    MS_20  = 20,
    MS_25  = 25,
    MS_40  = 40,
    MS_50  = 50,
    MS_100 = 100,
    MS_200 = 200,
  };

  // ---------------------------------------------------------------------------
  // EN pin active level / hold behavior (CMD_SET_EN_ACTIVE / 0x85, byte2 "enable")
  // ---------------------------------------------------------------------------
  enum class ENActiveMode : uint8_t {
    ActiveLow_L  = 0x00, // default in manual
    ActiveHigh_H = 0x01,
    AlwaysHold   = 0x02, // "always keep in the hold state"
  };

  // ---------------------------------------------------------------------------
  // Motor direction (CMD_SET_DIR / 0x86, byte2 "dir")
  // ---------------------------------------------------------------------------
  enum class MotorDirection : uint8_t {
    CW  = 0x00, // default in manual
    CCW = 0x01,
  };

  // ---------------------------------------------------------------------------
  // Pulse delay (CMD_SET_PULSE_DELAY / 0x87, byte2 "delay")
  // ---------------------------------------------------------------------------
  enum class PulseDelay : uint8_t {
    Delay_0ms  = 0x00, // 0ms
    Delay_4ms  = 0x01, // 4ms
    Delay_20ms = 0x02, // 20ms (default)
    Delay_40ms = 0x03, // 40ms
  };

  // ---------------------------------------------------------------------------
  // Stall protection enable (CMD_SET_STALL_PROTECT_ENABLE / 0x88, byte2 "enable")
  // ---------------------------------------------------------------------------
  enum class StallProtectEnable : uint8_t {
    Disable = 0x00, // default in manual
    Enable  = 0x01,
  };

  // Read stall state (CMD_READ_STALL_STATE / 0x3E, byte2 "status")
  enum class StallProtectState : uint8_t {
    NotProtected = 0x00,
    Protected    = 0x01,
  };

  // Release stall protection (CMD_RELEASE_STALL_PROTECT / 0x3D, byte2 "status")
  // Uses AckStatus semantics: 1=release success, 0=release fail.

  // ---------------------------------------------------------------------------
  // CAN bitrate (CMD_SET_CAN_BITRATE / 0x8A, byte2 "bitRate")
  // ---------------------------------------------------------------------------
  enum class CanBitrate : uint8_t {
    Kbps125 = 0x00,
    Kbps250 = 0x01,
    Kbps500 = 0x02, // default in manual
    Mbps1   = 0x03,
  };

  // ---------------------------------------------------------------------------
  // CAN response + active reporting (CMD_SET_RESPOND_ACTIVE / 0x8C)
  //   byte2 respon: 0=no response mode, 1=response mode
  //   byte3 active: 0=disable active report, 1=enable active report (default 1)
  // ---------------------------------------------------------------------------
  enum class RespondMode : uint8_t {
    NoResponse = 0x00,
    Response   = 0x01,
  };

  enum class ActiveReport : uint8_t {
    Disabled = 0x00,
    Enabled  = 0x01,
  };

  // ---------------------------------------------------------------------------
  // Axis lock (CMD_LOCK_AXIS / 0x8F, byte2 enable)
  // ---------------------------------------------------------------------------
  enum class AxisLock : uint8_t {
    Unlock = 0x00,
    Lock   = 0x01, // default in manual
  };

  // ---------------------------------------------------------------------------
  // Write IO port (CMD_WRITE_IO_PORT / 0x36)
  //   byte2 alm_mask, byte3 pend_mask:
  //     0 = do not write (keep original)
  //     1 = write
  //     2 = always keep (hold) (manual wording: "always keep original")
  //   byte4 ALM, byte5 PEND:
  //     0 = low, 1 = high
  // ---------------------------------------------------------------------------
  enum class IOWriteMask : uint8_t {
    KeepOriginal = 0x00,
    Write        = 0x01,
    HoldOriginal = 0x02,
  };

  enum class IOLevel : uint8_t {
    Low  = 0x00,
    High = 0x01,
  };

  // Read IO status (CMD_READ_IO_STATUS / 0x34) status bits:
  //   bit3: ALM, bit2: PEND, bit1: IN_2 (or EN for SERVO42E), bit0: IN_1
  static constexpr uint8_t IO_STATUS_BIT_IN1  = 0x01;
  static constexpr uint8_t IO_STATUS_BIT_IN2  = 0x02; // IN_2 (SERVO42D/57D) or EN (SERVO42E)
  static constexpr uint8_t IO_STATUS_BIT_PEND = 0x04;
  static constexpr uint8_t IO_STATUS_BIT_ALM  = 0x08;

  // Interpreting PEND/ALM bits in IO status (manual table)
  enum class PendState : uint8_t {
    NotInPlace     = 0x00, // PEND=0
    AlreadyInPlace = 0x01, // PEND=1
  };

  enum class AlmState : uint8_t {
    // Manual table: ALM=1 -> "No alarm", ALM=0 -> "Alarmed"
    Alarmed = 0x00,
    NoAlarm = 0x01,
  };

  // ---------------------------------------------------------------------------
  // Homing (CMD_SET_HOME_PARAM / 0x90)
  // ---------------------------------------------------------------------------
  enum class HomeTrigLevel : uint8_t {
    Low  = 0x00, // default
    High = 0x01,
  };

  enum class HomeDirection : uint8_t {
    CW  = 0x00, // default
    CCW = 0x01,
  };

  enum class EndstopLimitEnable : uint8_t {
    Disable = 0x00, // default
    Enable  = 0x01,
  };

  // CMD_GO_HOME / 0x91 status byte
  enum class GoHomeStatus : uint8_t {
    Fail    = 0x00,
    Start   = 0x01,
    Success = 0x02,
  };

  // ---------------------------------------------------------------------------
  // Set current axis to zero (CMD_SET_AXIS_ZERO / 0x92) uses AckStatus (0/1).
  // Set nolimit home current (CMD_SET_NOLIMIT_HOME_CURRENT / 0x93) uses AckStatus.
  // ---------------------------------------------------------------------------

  // ---------------------------------------------------------------------------
  // Set nolimit home param (CMD_SET_NOLIMIT_HOME_PARAM / 0x94)
  // ---------------------------------------------------------------------------
  enum class NoLimitHomeMode : uint8_t {
    UseLimitSwitch   = 0x00, // mode 0
    UseNoLimitSwitch = 0x01, // mode 1 (default)
  };

  enum class NoLimitHomeTrigger : uint8_t {
    Invalid_Disabled = 0x00, // trig 0
    AutoAfterPowerOn = 0x01, // trig 1 (default)
    EnPinTriggered   = 0x02, // trig 2
  };


  // No-limit homing "retValue" heuristic (CMD_SET_NOLIMIT_HOME_PARAM / 0x94)
  // Manual: default 0x2000; if speed > 500rpm set it >= 0x4000.
  static constexpr uint16_t NOLIMIT_HOME_DEFAULT_RET_VALUE = 0x2000;
  static constexpr uint16_t NOLIMIT_HOME_RECOMMENDED_RET_VALUE_FAST = 0x4000;
  // ---------------------------------------------------------------------------
  // Remap limit port (CMD_REMAP_LIMIT_PORT / 0x9E)
  // ---------------------------------------------------------------------------
  enum class LimitPortRemap : uint8_t {
    Disable = 0x00, // default
    Enable  = 0x01,
  };

  // ---------------------------------------------------------------------------
  // PEND pulse division output (CMD_SET_PULSE_DIV_OUTPUT / 0x9F)
  // ---------------------------------------------------------------------------
  enum class PulseDivStartLevel : uint8_t {
    StartLow  = 0x00, // default
    StartHigh = 0x01,
  };

  static constexpr uint16_t PEND_DIV_MIN_PERIOD_FOR_OUTPUT = 100; // manual: <100 = no output

  // ---------------------------------------------------------------------------
  // Save/Clean speed mode parameters (CMD_SAVE_CLEAN_SPEEDMODE / 0xFF, byte2 "state")
  // ---------------------------------------------------------------------------
  enum class SpeedModeParamState : uint8_t {
    Save  = 0xC8,
    Clean = 0xCA,
  };

  // ---------------------------------------------------------------------------
  // Bus-control query + run status
  // ---------------------------------------------------------------------------

  // Query motor status (CMD_QUERY_STATUS / 0xF1, byte2 "status")
  enum class MotorRunState : uint8_t {
    QueryFail    = 0x00,
    Stop         = 0x01,
    SpeedUp      = 0x02,
    SpeedDown    = 0x03,
    FullSpeed    = 0x04,
    Homing       = 0x05,
    Calibrating  = 0x06,
  };

  // Enable motor (CMD_ENABLE_BUS / 0xF3, byte2 "en")
  enum class BusEnable : uint8_t {
    Disable = 0x00,
    Enable  = 0x01,
  };

  // Emergency stop (CMD_EMERGENCY_STOP / 0xF7, byte2 "status") uses AckStatus semantics.

  // Speed mode (CMD_SPEED_MODE / 0xF6, uplink status byte uses AckStatus semantics: 1=run success, 0=run fail).

  // Position modes (CMD_POS_MODE1_REL_PULSES / 0xFD, CMD_POS_MODE2_ABS_PULSES / 0xFE,
  //                 CMD_POS_MODE3_REL_AXIS / 0xF4, CMD_POS_MODE4_ABS_AXIS / 0xF5)
  enum class PositionStatus : uint8_t {
    RunFail         = 0x00,
    RunStarting     = 0x01,
    RunComplete     = 0x02,
    EndLimitStopped = 0x03,
  };

  // Bit layout for speed/position mode "byte2" in bus-control commands (F6/FD/FE):
  static constexpr uint8_t BUS_DIR_BIT            = 0x80; // 0=forward, 1=reverse (see manual examples)
  static constexpr uint8_t BUS_SPEED_HI_NIBBLE_MSK = 0x0F; // lower 4 bits of byte2 contribute to speed[11:8]

  // ---------------------------------------------------------------------------
  // Calibrate encoder (CMD_CALIBRATE_ENCODER / 0x80, uplink status byte)
  // ---------------------------------------------------------------------------
  enum class CalibrateEncoderStatus : uint8_t {
    Calibrating = 0x00,
    Success     = 0x01,
    Fail        = 0x02,
  };

  // ---------------------------------------------------------------------------
  // Read EN pin status (CMD_READ_EN_STATUS / 0x3A, byte2 "enable")
  // ---------------------------------------------------------------------------
  enum class ENPinStatus : uint8_t {
    Disabled = 0x00,
    Enabled  = 0x01,
  };

  // ---------------------------------------------------------------------------
  // Version info (CMD_READ_VERSION_INFO / 0x40)
  //
  // byte2:
  //   bit7: series (1=E series, 0=D series)
  //   bit5..bit4: cal (1=CW: encoder increases, 2=CW: encoder decreases)
  //   bit3..bit0: hardVer
  // ---------------------------------------------------------------------------
  static constexpr uint8_t VERSION_SERIES_BIT     = 0x80;
  static constexpr uint8_t VERSION_CAL_MASK       = 0x30;
  static constexpr uint8_t VERSION_CAL_SHIFT      = 4;
  static constexpr uint8_t VERSION_HARDVER_MASK   = 0x0F;

  enum class MotorSeries : uint8_t {
    D = 0x00,
    E = 0x01,
  };

  // cal field (2-bit) meaning (manual lists values 1 and 2)
  enum class EncoderCalSense : uint8_t {
    CW_Increases = 0x01,
    CW_Decreases = 0x02,
  };

  // hardVer mapping table from manual
  enum class HardwareVersion : uint8_t {
    SERVO42E_RS485            = 0x01,
    SERVO42E_MKSPLCAN         = 0x02,
    SERVO57E_RS485            = 0x03,
    SERVO57E_MKSPLCAN         = 0x04,
    SERVO42D_RS485            = 0x05,
    SERVO42D_MKSPLCAN         = 0x06,
    SERVO57D_RS485            = 0x07,
    SERVO57D_MKSPLCAN         = 0x08,
  };

  // ---------------------------------------------------------------------------
  // Convenience: common "set" status byte patterns the manual repeats
  // ---------------------------------------------------------------------------
  static constexpr uint8_t STATUS_FAIL    = 0x00;
  static constexpr uint8_t STATUS_SUCCESS = 0x01;

} // namespace MKS
