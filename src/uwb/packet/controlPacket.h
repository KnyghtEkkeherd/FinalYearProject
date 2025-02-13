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

void print_packet_header(PacketHeader header)
{
  printf("Message Type: %s\n", mt_t_s(header.mt));
  printf("Packet Boundary Flag: %s\n", pbf_t_s(header.pbf));
}

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

// Function to send a ControlPackets
int send_packet(ControlPacket packet)
{
  printf("Sending packet...\n");
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

ControlPacket rcv_packet(uint8_t *buffer, size_t buffer_size, uint8_t *payload, uint8_t *payload_len)
{
  ControlPacket packet;

  printf("Receiving response...\n");
  int bytes_read = tty_rcv(buffer, buffer_size);
  if (bytes_read < 4)
  {
    printf("Failed to receive a valid response header\n");
    return packet; // Failed to receive valid response
  }

  // Print the header
  printf("Received Header: ");
  for (int i = 0; i < 4; i++)
  {
    printf("%02X ", buffer[i]);
  }
  printf("\n");

  // Translate into ControlPacket
  packet.header.mt = (buffer[0] >> 5) & 0b00000111;
  packet.header.pbf = (buffer[0] >> 4) & 0b00000001;
  packet.gid = (buffer[0] & 0b00001111);
  packet.oid = (buffer[1] & 0b00111111);
  packet.payload_len = buffer[3];

  print_packet_header(packet.header);

  // Extract payload length from the header
  *payload_len = buffer[3];

  // Print the payload
  printf("Received Payload: ");
  for (int i = 4; i < 4 + *payload_len; i++)
  {
    printf("%02X ", buffer[i]);
  }
  printf("\n");

  // Copy the payload
  memcpy(payload, buffer + 4, *payload_len);

  return packet;
}