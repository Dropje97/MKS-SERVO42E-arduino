#pragma once
// Selects the active CAN adapter for the current board.
// To add a new adapter (e.g. MCP2515 or other boards):
// 1) Add its header to src/transport/adapters/.
// 2) Add an #elif branch below that includes it and assigns CanBusAdapter.

#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
#include "UnoR4CanBus.h"
#include "../BufferedCanBus.h"
using CanBusAdapter = BufferedCanBus<UnoR4CanBus>;
#else
#error "No supported CAN adapter selected. Add a new adapter in src/transport/adapters/ and extend AdapterSelector.h."
#endif
