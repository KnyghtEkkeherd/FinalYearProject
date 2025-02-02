#ifndef CP_H_
#define CP_H_

#include <stdint.h>
#include "../../config/messages_id.h"
#include "../mt.h"
#include "../pbf.h"

#define MAX_PAYLOAD_SIZE 255

// Control Packet Structure
typedef struct
{
  MessageType mt : 3;                // Message Type (3 bits)
  PacketBoundaryFlag pbf : 1;        // Packet Boundary Flag (1 bit)
  uint8_t gid : 4;                   // Group Identifier (4 bits)
  uint8_t reserved1 : 2;             // Reserved for Future Use (2 bits)
  uint8_t oid : 6;                   // Opcode Identifier (6 bits)
  uint8_t reserved2;                 // Reserved for Future Use (8 bits)
  uint8_t payload_length;            // Payload Length (8 bits)
  uint8_t payload[MAX_PAYLOAD_SIZE]; // Payload
} ControlPacket;

// Function to validate a Control Packet
static inline int is_valid_control_packet(const ControlPacket *packet)
{
  return (packet && is_valid_message_type(packet->mt) && is_valid_pbf(packet->pbf) && packet->payload_length <= MAX_PAYLOAD_SIZE);
}

// Function to initialize a Control Packet
static inline void init_control_packet(ControlPacket *packet, MessageType mt, PacketBoundaryFlag pbf, uint8_t gid, uint8_t oid, uint8_t payload_length, const uint8_t *payload)
{
  // Ensure input parameters are valid
  if (!is_valid_message_type(mt) || !is_valid_pbf(pbf) || payload_length > MAX_PAYLOAD_SIZE)
  {
    return;
  }

  packet->mt = mt;
  packet->pbf = pbf;
  packet->gid = gid & 0x0F; // Mask to 4 bits
  packet->reserved1 = 0;    // Set reserved bits to 0
  packet->oid = oid & 0x3F; // Mask to 6 bits
  packet->reserved2 = 0;    // Set reserved bits to 0
  packet->payload_length = payload_length;

  // Copy the payload if provided
  if (payload && payload_length > 0)
  {
    for (uint8_t i = 0; i < payload_length; i++)
    {
      packet->payload[i] = payload[i];
    }
  }
}

// Create a Control Packet from a buffer
static inline void parse_control_packet(ControlPacket *packet, const uint8_t *buffer, size_t len)
{
  if (!packet || !buffer || len < 4)
  {
    return;
  }

  packet->mt = (buffer[0] >> 5) & 0x07;  // Message Type (3 bits)
  packet->pbf = (buffer[0] >> 4) & 0x01; // Packet Boundary Flag (1 bit)
  packet->gid = buffer[0] & 0x0F;        // Group Identifier (4 bits)
  packet->reserved1 = 0;                 // Reserved for Future Use (2 bits)
  packet->oid = buffer[1] & 0x3F;        // Opcode Identifier (6 bits)
  packet->reserved2 = 0;                 // Reserved for Future Use (8 bits)
  packet->payload_length = buffer[3];    // Payload Length (8 bits)

  // Copy the payload if provided
  if (packet->payload_length > 0)
  {
    for (uint8_t i = 0; i < packet->payload_length; i++)
    {
      packet->payload[i] = buffer[4 + i];
    }
  }
}

// Print the contents of a Control Packet
static inline void print_control_packet(const ControlPacket *packet)
{
  if (!packet)
  {
    return;
  }

  printf("Control Packet:\n");
  printf("MT: %d\n", packet->mt);
  printf("PBF: %d\n", packet->pbf);
  printf("Group Identifier: %d (%s)\n", packet->gid, get_gid_description(packet->gid));
  printf("Opcode Identifier: %d (%s)\n", packet->oid, get_oid_description(packet->gid, packet->oid));
  printf("Payload Length: %d\n", packet->payload_length);
  printf("Payload: ");
  for (uint8_t i = 0; i < packet->payload_length; i++)
  {
    printf("%02X ", packet->payload[i]);
  }
  printf("\n");
}

#endif // CP_H_
