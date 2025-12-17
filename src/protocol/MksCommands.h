#pragma once
#include <stdint.h>

// Command byte (byte1) values from "MKS SERVO42E/57E CAN User Manual V1.0.1"
namespace MKS {
  // Read commands
  static constexpr uint8_t CMD_READ_ENCODER_CARRY       = 0x30; // Read the encoder carry/position value (encoder count + 14-bit angle).
  static constexpr uint8_t CMD_READ_ENCODER_ADDITION    = 0x31; // Read the encoder addition value (int48), where each full turn changes by ±0x4000.
  static constexpr uint8_t CMD_READ_SPEED_RPM           = 0x32; // Read the real-time motor speed in RPM (CCW > 0, CW < 0).
  static constexpr uint8_t CMD_READ_INPUT_PULSES        = 0x33; // Read the number of input pulses received (int32).
  static constexpr uint8_t CMD_READ_IO_STATUS           = 0x34; // Read IO port status bits (IN_1/IN_2, PEND in-place, and ALM alarm).
  static constexpr uint8_t CMD_READ_POS_ERROR           = 0x39; // Read the motor shaft angle error (target minus real-time angle, scaled 0–51200 → 0–360°).
  static constexpr uint8_t CMD_READ_EN_STATUS           = 0x3A; // Read the EN pin status (1=enabled, 0=disabled).
  static constexpr uint8_t CMD_RELEASE_STALL_PROTECT    = 0x3D; // Release/clear the locked-rotor (stall) protection state.
  static constexpr uint8_t CMD_READ_STALL_STATE         = 0x3E; // Read the locked-rotor (stall) protection state (1=protected, 0=not protected).
  static constexpr uint8_t CMD_RESTORE_DEFAULTS         = 0x3F; // Restore factory-default parameters (the motor reboots after restoring).
  static constexpr uint8_t CMD_READ_VERSION_INFO        = 0x40; // Read series/calibration flag plus hardware and firmware version fields.
  static constexpr uint8_t CMD_RESTART                  = 0x41; // Restart the motor/controller (does not modify configuration parameters).
  static constexpr uint8_t CMD_USER_ID                  = 0x42; // Read or write the 32-bit USER ID (direction depends on frame type). read/write depends on frame direction
  static constexpr uint8_t CMD_WRITE_USER_ID            = CMD_USER_ID; // Alias for CMD_USER_ID when writing the USER ID.
  static constexpr uint8_t CMD_READ_USER_ID             = CMD_USER_ID; // Alias for CMD_USER_ID when reading the USER ID.

  // Parameter read (special)
  static constexpr uint8_t CMD_READ_PARAM               = 0x00; // Read a system parameter specified by the following code byte (e.g., 0x82 for work mode).

  // Write/Set parameter commands
  static constexpr uint8_t CMD_WRITE_IO_PORT            = 0x36; // Write ALM/PEND output values using per-port masks (only masked ports are updated).
  static constexpr uint8_t CMD_CALIBRATE_ENCODER        = 0x80; // Calibrate the motor-direction ↔ encoder-direction relationship (status: 0=busy, 1=ok, 2=fail).
  static constexpr uint8_t CMD_SET_MODE                 = 0x82; // Set the work mode (pulse control open/closed-loop or bus control).
  static constexpr uint8_t CMD_SET_CURRENT_MA           = 0x83; // Set the working current in mA.
  static constexpr uint8_t CMD_SET_MICROSTEP            = 0x84; // Set the microstep subdivision value (0=256; other supported values per manual).
  static constexpr uint8_t CMD_SET_EN_ACTIVE            = 0x85; // Set the effective (active) level/polarity of the EN pin.
  static constexpr uint8_t CMD_SET_DIR                  = 0x86; // Set motor running direction (0=CW, 1=CCW) for control commands.
  static constexpr uint8_t CMD_SET_PULSE_DELAY          = 0x87; // Set the pulse/dir delay time (0/4/20/40 ms).
  static constexpr uint8_t CMD_SET_STALL_PROTECT_ENABLE = 0x88; // Enable or disable locked-rotor (stall) protection.
  static constexpr uint8_t CMD_SET_STALL_TOLERANCE      = 0x89; // Set the stall tolerance threshold (angle error) that triggers stall protection.
  static constexpr uint8_t CMD_SET_CAN_BITRATE          = 0x8A; // Set the CAN bitrate (125k/250k/500k/1M).
  static constexpr uint8_t CMD_SET_CAN_ID               = 0x8B; // Set the CAN slave ID (0=broadcast, 1…0x7FF).
  static constexpr uint8_t CMD_SET_RESPOND_ACTIVE       = 0x8C; // Configure whether the slave responds to commands and/or actively reports status in bus mode.
  static constexpr uint8_t CMD_SET_GROUP_ID             = 0x8D; // Set the group ID used for group addressing (no response when group address is used).
  static constexpr uint8_t CMD_LOCK_AXIS                = 0x8F; // Set whether to lock/hold the axis when using bus control.

  // Homing
  static constexpr uint8_t CMD_SET_HOME_PARAM           = 0x90; // Configure homing parameters (trigger level, direction, speed, end-limit enable, and home mode).
  static constexpr uint8_t CMD_GO_HOME                  = 0x91; // Start the go-home routine (also used as a 'limit reset' after changing limit parameters).
  static constexpr uint8_t CMD_SET_AXIS_ZERO            = 0x92; // Set the current axis position as zero without moving.
  static constexpr uint8_t CMD_SET_NOLIMIT_HOME_CURRENT = 0x93; // Set the current/torque used during no-limit (stall-based) homing.
  static constexpr uint8_t CMD_SET_NOLIMIT_HOME_PARAM   = 0x94; // Set no-limit homing parameters (reverse angle retValue and homing current).
  static constexpr uint8_t CMD_REMAP_LIMIT_PORT         = 0x9E; // Enable/disable remapping the EN port as the right limit port (bus control only; not supported on 57E).
  static constexpr uint8_t CMD_SET_PULSE_DIV_OUTPUT     = 0x9F; // Map the PEND port as a pulse frequency-division output (set start level and division period; 0 disables). Maps PEND as pulse division output

  // Bus-control run commands
  static constexpr uint8_t CMD_QUERY_STATUS             = 0xF1; // Query the current bus-mode running state (stopped/speeding up/down/full speed/homing/calibrating).
  static constexpr uint8_t CMD_ENABLE_BUS               = 0xF3; // Enable or disable the motor for bus control mode (1=enable, 0=disable).
  static constexpr uint8_t CMD_POS_MODE3_REL_AXIS       = 0xF4; // Position mode 3: relative move by axis (encoder addition) using the given speed and acceleration.
  static constexpr uint8_t CMD_POS_MODE4_ABS_AXIS       = 0xF5; // Position mode 4: absolute move to an axis (encoder addition) with speed/acc (supports real-time updates).
  static constexpr uint8_t CMD_SPEED_MODE               = 0xF6; // Speed mode: run at target speed with acceleration (speed=0 stops).
  static constexpr uint8_t CMD_EMERGENCY_STOP           = 0xF7; // Emergency stop the motor immediately in bus control mode.

  static constexpr uint8_t CMD_POS_MODE1_REL_PULSES     = 0xFD; // Position mode 1: relative move by pulses with the given speed and acceleration.
  static constexpr uint8_t CMD_POS_MODE2_ABS_PULSES     = 0xFE; // Position mode 2: absolute move to a pulse position with the given speed and acceleration.

  static constexpr uint8_t CMD_SAVE_CLEAN_SPEEDMODE     = 0xFF; // Save (0xC8) or clear (0xCA) the stored speed-mode parameters.
}
