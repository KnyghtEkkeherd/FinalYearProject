#include <string.h>
#include "header/header.h"
#include "identifiers/gid.h"
#include "identifiers/oid.h"
#include "../transport/tty.h"

#define MAX_PAYLOAD_SIZE 255

typedef struct
{
  PacketHeader header;
  GroupIdentifier gid;
  OpcodeIdentifier oid;
  uint8_t payload_len;
  uint8_t payload[MAX_PAYLOAD_SIZE];
} ControlPacket;

ControlPacket create_packet(MessageType mt, PacketBoundaryFlag pbf, GroupIdentifier gid, OpcodeIdentifier oid, uint8_t *payload, uint8_t payload_len)
{
  ControlPacket packet;
  packet.header.mt = mt;
  packet.header.pbf = pbf;
  packet.gid = gid;
  packet.oid = oid;
  packet.payload_len = payload_len;
  memcpy(packet.payload, payload, payload_len);
  return packet;
}

// Function to send a ControlPacket
int send_packet(ControlPacket packet)
{
  uint8_t buffer[4 + packet.payload_len];

  // Octet 0: first 3 bits are mt, next bit is pbf, next 4 are gid
  buffer[0] = ((packet.header.mt & 0b00000111) << 5) |  // mask and shift the first 3 bits of mt
              ((packet.header.pbf & 0b00000001) << 4) | // mask and shift the pbf bit
              (packet.gid & 0b00001111);                // mask the gid bits

  // Octet 1: first 2 bits are rfu, next 6 bits are oid
  buffer[1] = (packet.oid & 0b00111111); // mask the oid bits

  // Octet 2: rfu
  buffer[2] = 0b00000000; // reserved for future use

  // Octet 3: payload length
  buffer[3] = packet.payload_len;

  // Octet 4 to 4 + payload length: payload
  memcpy(buffer + 4, packet.payload, packet.payload_len);

  return tty_send(buffer, sizeof(buffer));
}
