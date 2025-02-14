#include <stdio.h>

#ifndef STATUS_H
#define STATUS_H

enum UciStatus
{
  STATUS_OK = 0x00,
  STATUS_REJECTED = 0x01,
  STATUS_FAILED = 0x02,
  STATUS_SYNTAX_ERROR = 0x03,
  STATUS_INVALID_PARAM = 0x04,
  STATUS_INVALID_RANGE = 0x05,
  STATUS_INVALID_MSG_SIZE = 0x06,
  STATUS_UNKNOWN_GID = 0x07,
  STATUS_UNKNOWN_OID = 0x08,
  STATUS_READ_ONLY = 0x09,
  STATUS_COMMAND_RETRY = 0x0A,
  STATUS_SESSION_NOT_EXIST = 0x11,
  STATUS_SESSION_DUPLICATE = 0x12,
  STATUS_SESSION_ACTIVE = 0x13,
  STATUS_MAX_SESSIONS_EXCEEDED = 0x14,
  STATUS_SESSION_NOT_CONFIGURED = 0x15,
  STATUS_ACTIVE_SESSIONS_ONGOING = 0x16,
  STATUS_MULTICAST_LIST_FULL = 0x17,
  STATUS_ADDRESS_NOT_FOUND = 0x18,
  STATUS_ADDRESS_ALREADY_PRESENT = 0x19,
  STATUS_RANGING_TX_FAILED = 0x20,
  STATUS_RANGING_RX_TIMEOUT = 0x21,
  STATUS_RANGING_RX_PHY_DEC_FAILED = 0x22,
  STATUS_RANGING_RX_PHY_TOA_FAILED = 0x23,
  STATUS_RANGING_RX_PHY_STS_FAILED = 0x24,
  STATUS_RANGING_RX_MAC_DEC_FAILED = 0x25,
  STATUS_RANGING_RX_MAC_IE_DEC_FAILED = 0x26,
  STATUS_RANGING_RX_MAC_IE_MISSING = 0x27,
  REASON_RESET_POWER_CYCLE = 0x28,
  REASON_RESET_SOFTWARE = 0x29,
  REASON_RESET_WATCHDOG = 0x2A,
  REASON_RESET_BROWNOUT = 0x2B,
};

const char *status_to_s(enum UciStatus status)
{
  switch (status)
  {
  case STATUS_OK:
    return ("Operation completed successfully.");
    break;
  case STATUS_REJECTED:
    return ("Operation was rejected.");
    break;
  case STATUS_FAILED:
    return ("Operation failed.");
    break;
  case STATUS_SYNTAX_ERROR:
    return ("Syntax error in the command.");
    break;
  case STATUS_INVALID_PARAM:
    return ("Invalid parameter provided.");
    break;
  case STATUS_INVALID_RANGE:
    return ("Parameter value out of range.");
    break;
  case STATUS_INVALID_MSG_SIZE:
    return ("Invalid message size.");
    break;
  case STATUS_UNKNOWN_GID:
    return ("Unknown Group Identifier.");
    break;
  case STATUS_UNKNOWN_OID:
    return ("Unknown Opcode Identifier.");
    break;
  case STATUS_READ_ONLY:
    return ("Attempt to write to a read-only parameter.");
    break;
  case STATUS_COMMAND_RETRY:
    return ("Command should be retried.");
    break;
  case STATUS_SESSION_NOT_EXIST:
    return ("Session does not exist.");
    break;
  case STATUS_SESSION_DUPLICATE:
    return ("Duplicate session.");
    break;
  case STATUS_SESSION_ACTIVE:
    return ("Session is already active.");
    break;
  case STATUS_MAX_SESSIONS_EXCEEDED:
    return ("Maximum number of sessions exceeded.");
    break;
  case STATUS_SESSION_NOT_CONFIGURED:
    return ("Session is not configured.");
    break;
  case STATUS_ACTIVE_SESSIONS_ONGOING:
    return ("Active sessions are ongoing.");
    break;
  case STATUS_MULTICAST_LIST_FULL:
    return ("Multicast list is full.");
    break;
  case STATUS_ADDRESS_NOT_FOUND:
    return ("Address not found.");
    break;
  case STATUS_ADDRESS_ALREADY_PRESENT:
    return ("Address is already present.");
    break;
  case STATUS_RANGING_TX_FAILED:
    return ("Ranging transmission failed.");
    break;
  case STATUS_RANGING_RX_TIMEOUT:
    return ("Ranging reception timed out.");
    break;
  case STATUS_RANGING_RX_PHY_DEC_FAILED:
    return ("Ranging reception PHY decode failed.");
    break;
  case STATUS_RANGING_RX_PHY_TOA_FAILED:
    return ("Ranging reception PHY TOA failed.");
    break;
  case STATUS_RANGING_RX_PHY_STS_FAILED:
    return ("Ranging reception PHY STS failed.");
    break;
  case STATUS_RANGING_RX_MAC_DEC_FAILED:
    return ("Ranging reception MAC decode failed.");
    break;
  case STATUS_RANGING_RX_MAC_IE_DEC_FAILED:
    return ("Ranging reception MAC IE decode failed.");
    break;
  case STATUS_RANGING_RX_MAC_IE_MISSING:
    return ("Ranging reception MAC IE missing.");
    break;
  case REASON_RESET_POWER_CYCLE:
    return ("Reset due to power cycle.");
    break;
  case REASON_RESET_SOFTWARE:
    return ("Reset due to software issue.");
    break;
  case REASON_RESET_WATCHDOG:
    return ("Reset due to watchdog timer.");
    break;
  case REASON_RESET_BROWNOUT:
    return ("Reset due to brownout condition.");
    break;
  default:
    return ("Unknown status code");
    break;
  }
}

#endif // STATUS_H