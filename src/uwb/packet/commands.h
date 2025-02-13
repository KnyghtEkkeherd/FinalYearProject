#include "../transport/tty.h"
#include "controlPacket.h"

uint8_t buffer[MAX_PAYLOAD_SIZE + 4];
uint8_t payload[MAX_PAYLOAD_SIZE];
uint8_t payload_len;

int reset_device()
{
  // Create the reset packet (assuming the reset command is 0x00 in the UCI core group)
  uint8_t reason = 0x00; // Example reason for reset
  ControlPacket packet = create_packet(COMMAND, COMPLETE, CORE, CORE_DEVICE_RESET, &reason, 1);

  printf("Sending reset command...\n");
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  // Receive the response
  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer), payload, &payload_len);
  if (rcvd_packet.gid != CORE || rcvd_packet.oid != CORE_DEVICE_RESET)
  {
    return -1;
  }

  return 0;
}

int get_device_info()
{
  // Create the CORE_GET_DEVICE_INFO packet
  ControlPacket packet = create_packet(COMMAND, COMPLETE, CORE, CORE_GET_DEVICE_INFO, NULL, 0);

  // Send the CORE_GET_DEVICE_INFO command
  printf("Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1; // Failed to send command
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer), payload, &payload_len);
  if (rcvd_packet.gid)
  {
    return -1; // Failed to receive valid response
  }

  return 0; // Success
}

int init_uwb_session(uint32_t sid, uint8_t stype)
{
  // Create the SESSION_INIT packet
  uint8_t session_init_payload[5];
  session_init_payload[0] = (uint8_t)(sid & 0xFF);
  session_init_payload[1] = (uint8_t)((sid >> 8) & 0xFF);
  session_init_payload[2] = (uint8_t)((sid >> 16) & 0xFF);
  session_init_payload[3] = (uint8_t)((sid >> 24) & 0xFF);
  session_init_payload[4] = stype;

  ControlPacket packet = create_packet(COMMAND, COMPLETE, SESSION, SESSION_INIT, session_init_payload, 5);

  // Send the SESSION_INIT command
  printf("Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1; // Failed to send command
  }

  // Receive the response
  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer), payload, &payload_len);
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != SESSION_INIT)
  {
    printf("Unexpected response received: GID: 0x%02x, OID: 0x%02x\n", rcvd_packet.gid, rcvd_packet.oid);
    return -1; // Failed to receive valid response
  }

  if (payload_len > 0 && payload[0] == 0x03)
  {
    printf("Received Syntax Error response\n");
    return -1; // Syntax error
  }

  return 0; // Success
}

int set_uwb_session_parameters(uint32_t session_id)
{
  uint8_t session_params[] = {
      0x00, 0x01,       // DeviceType: Controller (0x00), DeviceRole: Initiator (0x01)
      0x06, 0x00, 0x01, // DeviceMacAddress: 0x00000001
      0x05, 0x01,       // NumberOfControlees: 1
      0x07, 0x00, 0x02, // DstMacAddress: 0x00000002
      0x04, 0x09,       // ChannelNumber: Channel 9
  };
  uint8_t session_params_len = sizeof(session_params);

  uint8_t payload[MAX_PAYLOAD_SIZE];
  int offset = 0;

  payload[offset++] = (uint8_t)(session_id & 0xFF);
  payload[offset++] = (uint8_t)((session_id >> 8) & 0xFF);
  payload[offset++] = (uint8_t)((session_id >> 16) & 0xFF);
  payload[offset++] = (uint8_t)((session_id >> 24) & 0xFF);

  for (int i = 0; i < session_params_len; i++)
  {
    payload[offset++] = session_params[i];
  }

  ControlPacket packet = create_packet(COMMAND, COMPLETE, SESSION, SESSION_SET_APP_CONFIG, payload, offset);

  printf("Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer), payload, &payload_len);
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != SESSION_SET_APP_CONFIG)
  {
    return -1;
  }

  return 0;
}

int start_uwb_ranging_session(uint32_t session_id)
{
  uint8_t payload[4];
  payload[0] = (uint8_t)(session_id & 0xFF);
  payload[1] = (uint8_t)((session_id >> 8) & 0xFF);
  payload[2] = (uint8_t)((session_id >> 16) & 0xFF);
  payload[3] = (uint8_t)((session_id >> 24) & 0xFF);

  ControlPacket packet = create_packet(COMMAND, COMPLETE, RANGING, RANGE_START, payload, 4);

  printf("Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer), payload, &payload_len);
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != RANGE_START)
  {
    return -1;
  }

  return 0;
}

int stop_uwb_ranging_session(uint32_t session_id)
{
  uint8_t payload[4];
  payload[0] = (uint8_t)(session_id & 0xFF);
  payload[1] = (uint8_t)((session_id >> 8) & 0xFF);
  payload[2] = (uint8_t)((session_id >> 16) & 0xFF);
  payload[3] = (uint8_t)((session_id >> 24) & 0xFF);

  ControlPacket packet = create_packet(COMMAND, COMPLETE, RANGING, RANGE_STOP, payload, 4);

  printf("Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer), payload, &payload_len);
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != RANGE_STOP)
  {
    return -1;
  }

  return 0;
}
