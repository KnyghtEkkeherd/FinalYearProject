#ifndef MESSAGES_ID_H_
#define MESSAGES_ID_H_

// Group Identifier (GID) Definitions
#define GID_UCI_CORE 0x0
#define GID_UWB_SESSION 0x1
#define GID_UWB_RANGING 0x2

// GID_UCI_CORE Opcodes (group 0)
#define CORE_DEVICE_RESET 0x00    // Reset device
#define CORE_DEVICE_STATUS 0x01   // Device status notification
#define CORE_GET_DEVICE_INFO 0x02 // Get device information
#define CORE_GET_CAPS_INFO 0x03   // Get device capabilities
#define CORE_SET_CONFIG 0x04      // Set device configuration
#define CORE_GET_CONFIG 0x05      // Get device configuration
#define CORE_GENERIC_ERROR 0x07   // Generic error notification

// GID_UWB_SESSION Opcodes (group 1)
#define SESSION_INIT 0x00              // Request to create a UWB session
#define SESSION_DEINIT 0x01            // Request to destroy a UWB session
#define SESSION_STATUS 0x02            // Notifies the current UWB session state
#define SESSION_SET_APP_CONFIG 0x03    // Configuration request for UWB session with specified configurations
#define SESSION_GET_APP_CONFIG 0x04    // To get the current configuration for specified UWB session
#define SESSION_GET_COUNT 0x05         // To get the number of UWB session created in UWBS device
#define SESSION_GET_STATE 0x06         // To get the current state of the UWB session
#define SESSION_UPDATE_CONTROLLER 0x07 // To update the multicast list

// GID_UWB_RANGING Opcodes (group 2)
#define RANGE_START 0x00 // To activate the UWB ranging session
#define RANGE_STOP 0x01  // To deactivate the ranging session
#define RFU 0x02         // RFU

// Function to get the description of a GID
const char *get_gid_description(uint8_t gid)
{
  switch (gid)
  {
  case GID_UCI_CORE:
    return "Core";
  case GID_UWB_SESSION:
    return "UWB Session";
  case GID_UWB_RANGING:
    return "UWB Ranging";
  default:
    return "Unknown GID";
  }
}

// Function to get the description of an OID based on GID
const char *get_oid_description(uint8_t gid, uint8_t oid)
{
  switch (gid)
  {
  case GID_UCI_CORE:
    switch (oid)
    {
    case CORE_DEVICE_RESET:
      return "Device Reset";
    case CORE_DEVICE_STATUS:
      return "Device Status";
    case CORE_GET_DEVICE_INFO:
      return "Get Device Info";
    case CORE_GET_CAPS_INFO:
      return "Get Device Capabilities";
    case CORE_SET_CONFIG:
      return "Set Device Configuration";
    case CORE_GET_CONFIG:
      return "Get Device Configuration";
    case CORE_GENERIC_ERROR:
      return "Generic Error";
    default:
      return "Unknown OID";
    }
  case GID_UWB_SESSION:
    switch (oid)
    {
    case SESSION_INIT:
      return "Session Init";
    case SESSION_DEINIT:
      return "Session Deinit";
    case SESSION_STATUS:
      return "Session Status";
    case SESSION_SET_APP_CONFIG:
      return "Set App Config";
    case SESSION_GET_APP_CONFIG:
      return "Get App Config";
    case SESSION_GET_COUNT:
      return "Get Session Count";
    case SESSION_GET_STATE:
      return "Get Session State";
    case SESSION_UPDATE_CONTROLLER:
      return "Update Controller";
    default:
      return "Unknown OID";
    }
  case GID_UWB_RANGING:
    switch (oid)
    {
    case RANGE_START:
      return "Range Start";
    case RANGE_STOP:
      return "Range Stop";
    case RFU:
      return "RFU";
    default:
      return "Unknown OID";
    }
  default:
    return "Unknown OID";
  }
}

#endif // MESSAGES_ID_H_
