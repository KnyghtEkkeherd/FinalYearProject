typedef enum
{
  // Core
  CORE_DEVICE_RESET = 0x0,
  CORE_DEVICE_STATUS = 0x1,
  CORE_GET_DEVICE_INFO = 0x2,
  // Session
  SESSION_INIT = 0x0,
  SESSION_DEINIT = 0x1,
  SESSION_STATUS = 0x2,
  // Ranging
  RANGE_START = 0x0,
  RANGE_STOP = 0x1
} OpcodeIdentifier;