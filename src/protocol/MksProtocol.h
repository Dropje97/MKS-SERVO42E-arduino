#pragma once
#include <stdint.h>

#include "protocol/MksCommands.h"
#include "protocol/MksEnums.h"

/*
  MksProtocol.h

  Single include to pull in protocol constants (commands + enums) for the
  MKS SERVO42E/57E ONLY_CAN protocol.

  Source document:
  "MKS SERVO42E/57E ONLY_CAN User Manual V1.0.1"
*/

namespace MKS {
  // Human-readable identifier for logs/debug prints.
  static constexpr const char* PROTOCOL_DOC =
      "MKS SERVO42E/57E ONLY_CAN User Manual V1.0.1";
} // namespace MKS
