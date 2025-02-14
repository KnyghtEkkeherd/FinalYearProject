#include <stdint.h>
#include <string.h>
#include "header/header.h"
#include "identifiers/gid.h"
#include "identifiers/oid.h"

#define MIN_PAYLOAD_SIZE 50
#define MAX_RANGING_MEASUREMENTS 10

typedef struct
{
  PacketHeader header;
  GroupIdentifier RANGING;
  OpcodeIdentifier RANGE_START;
  uint32_t SequenceNumber;
  uint32_t SessionID;
  uint8_t RCRIndication;
  uint32_t CurrentRangingInterval;
  uint8_t RangingMeasurementType;
  uint8_t MACAddressingModeIndicator;
  uint8_t NumberofRangingMeasurements;
  uint16_t MACAddress[MAX_RANGING_MEASUREMENTS];
  uint8_t Status[MAX_RANGING_MEASUREMENTS];
  uint8_t NLoS[MAX_RANGING_MEASUREMENTS];
  uint16_t Distance[MAX_RANGING_MEASUREMENTS];
} NotificationPacket;

NotificationPacket parse_notif(uint8_t *payload)
{
  NotificationPacket notif;
  // Assuming payload is at least MIN_PAYLOAD_SIZE bytes
  if (payload == NULL || notif.header.mt != NOTIF)
  {
    return notif;
  }

  notif.SequenceNumber = *((uint32_t *)(payload));
  notif.SessionID = *((uint32_t *)(payload + 4));
  notif.RCRIndication = *(payload + 8);
  notif.CurrentRangingInterval = *((uint32_t *)(payload + 9));
  notif.RangingMeasurementType = *(payload + 13);
  notif.MACAddressingModeIndicator = *(payload + 14);
  notif.NumberofRangingMeasurements = *(payload + 15);

  if (notif.NumberofRangingMeasurements > MAX_RANGING_MEASUREMENTS)
  {
    notif.NumberofRangingMeasurements = MAX_RANGING_MEASUREMENTS;
  }

  for (uint8_t i = 0; i < notif.NumberofRangingMeasurements; ++i)
  {
    uint16_t base_index = 16 + i * 7;
    notif.MACAddress[i] = *((uint16_t *)(payload + base_index));
    notif.Status[i] = *(payload + base_index + 2);
    notif.NLoS[i] = *(payload + base_index + 3);
    notif.Distance[i] = *((uint16_t *)(payload + base_index + 4));
  }

  return notif;
}