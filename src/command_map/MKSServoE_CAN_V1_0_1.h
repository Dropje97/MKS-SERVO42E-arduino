\
#pragma once
#include <stdint.h>

// Command byte (byte1) values from "MKS SERVO42E/57E CAN User Manual V1.0.1"
namespace MKS {
  // Read commands
  static constexpr uint8_t CMD_READ_ENCODER_CARRY      = 0x30;
  static constexpr uint8_t CMD_READ_ENCODER_ADDITION   = 0x31;
  static constexpr uint8_t CMD_READ_SPEED_RPM          = 0x32;
  static constexpr uint8_t CMD_READ_INPUT_PULSES       = 0x33;
  static constexpr uint8_t CMD_READ_IO_STATUS          = 0x34;
  static constexpr uint8_t CMD_READ_POS_ERROR          = 0x39;
  static constexpr uint8_t CMD_READ_EN_STATUS          = 0x3A;
  static constexpr uint8_t CMD_RELEASE_STALL_PROTECT   = 0x3D;
  static constexpr uint8_t CMD_READ_STALL_STATE        = 0x3E;
  static constexpr uint8_t CMD_RESTORE_DEFAULTS        = 0x3F;
  static constexpr uint8_t CMD_READ_VERSION_INFO       = 0x40;
  static constexpr uint8_t CMD_RESTART                 = 0x41;
  static constexpr uint8_t CMD_WRITE_USER_ID           = 0x42;
  static constexpr uint8_t CMD_READ_USER_ID            = 0x42;

  // Parameter read (special)
  static constexpr uint8_t CMD_READ_PARAM              = 0x00;

  // Write/Set parameter commands
  static constexpr uint8_t CMD_WRITE_IO_PORT           = 0x36;
  static constexpr uint8_t CMD_SET_MODE                = 0x82;
  static constexpr uint8_t CMD_SET_CURRENT_MA          = 0x83;
  static constexpr uint8_t CMD_SET_MICROSTEP           = 0x84;
  static constexpr uint8_t CMD_SET_EN_ACTIVE            = 0x85;
  static constexpr uint8_t CMD_SET_DIR                  = 0x86;
  static constexpr uint8_t CMD_SET_PULSE_DELAY          = 0x87;
  static constexpr uint8_t CMD_SET_STALL_PROTECT_ENABLE = 0x88;
  static constexpr uint8_t CMD_SET_STALL_TOLERANCE      = 0x89;
  static constexpr uint8_t CMD_SET_CAN_BITRATE          = 0x8A;
  static constexpr uint8_t CMD_SET_CAN_ID               = 0x8B;
  static constexpr uint8_t CMD_SET_RESPOND_ACTIVE       = 0x8C;
  static constexpr uint8_t CMD_SET_GROUP_ID             = 0x8D;
  static constexpr uint8_t CMD_LOCK_AXIS                = 0x8F;

  // Homing
  static constexpr uint8_t CMD_SET_HOME_PARAM           = 0x90;
  static constexpr uint8_t CMD_GO_HOME                  = 0x91;
  static constexpr uint8_t CMD_SET_AXIS_ZERO            = 0x92;
  static constexpr uint8_t CMD_SET_NOLIMIT_HOME_PARAM   = 0x94;
  static constexpr uint8_t CMD_REMAP_LIMIT_PORT         = 0x9E;
  static constexpr uint8_t CMD_PEND_DIV_OUTPUT          = 0x9F;

  // Bus-control run commands
  static constexpr uint8_t CMD_QUERY_STATUS             = 0xF1;
  static constexpr uint8_t CMD_ENABLE_BUS               = 0xF3;
  static constexpr uint8_t CMD_POS_MODE3_REL_AXIS       = 0xF4;
  static constexpr uint8_t CMD_POS_MODE4_ABS_AXIS       = 0xF5;
  static constexpr uint8_t CMD_SPEED_MODE               = 0xF6;
  static constexpr uint8_t CMD_EMERGENCY_STOP           = 0xF7;

  static constexpr uint8_t CMD_POS_MODE1_REL_PULSES     = 0xFD;
  static constexpr uint8_t CMD_POS_MODE2_ABS_PULSES     = 0xFE;

  static constexpr uint8_t CMD_SAVE_CLEAN_SPEEDMODE     = 0xFF;

  // Some ops return status code 0x93 in the manual (used as a generic status response in some sections).
  static constexpr uint8_t RSP_GENERIC_STATUS_93        = 0x93;
}
