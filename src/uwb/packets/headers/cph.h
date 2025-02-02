#ifndef CPH_H_
#define CPH_H_

#include <stdint.h>
#include "../mt.h"
#include "../pbf.h"

// Common Packet Header Structure
typedef struct
{
  MessageType mt : 3;         // Message Type (3 bits)
  PacketBoundaryFlag pbf : 1; // Packet Boundary Flag (1 bit)
  uint8_t reserved : 4;       // Reserved bits (4 bits, unused)
} CommonPacketHeader;

// Function to construct a Common Packet Header
static inline CommonPacketHeader create_common_packet_header(MessageType mt, PacketBoundaryFlag pbf)
{
  CommonPacketHeader header;
  if (is_valid_message_type(mt) && is_valid_pbf(pbf))
  {
    header.mt = mt;
    header.pbf = pbf;
  }
  return header;
}

// Function to validate a Common Packet Header
static inline int is_valid_common_packet_header(const CommonPacketHeader *header)
{
  return header && is_valid_message_type(header->mt) && is_valid_pbf(header->pbf);
}

#endif // CPH_H_