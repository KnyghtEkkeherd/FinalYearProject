#include "../transport/tty.h"
#include "controlPacket.h"

uint8_t buffer[MAX_PAYLOAD_SIZE + 4];

int reset_device()
{
  uint8_t temp_payload = REASON_RESET_POWER_CYCLE;
  ControlPacket packet = create_packet(COMMAND, COMPLETE, CORE, CORE_DEVICE_RESET, &temp_payload, 1);

  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  // Receive the response
  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != CORE || rcvd_packet.oid != CORE_DEVICE_RESET)
  {
    return -2;
  }

  return 1;
}

int get_device_info()
{
  // Create the CORE_GET_DEVICE_INFO packet
  ControlPacket packet = create_packet(COMMAND, COMPLETE, CORE, CORE_GET_DEVICE_INFO, NULL, 0);

  // Send the CORE_GET_DEVICE_INFO command
  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1; // Failed to send command
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid)
  {
    return -2; // Failed to receive valid response
  }

  // Parse device info
  int offset = 0;

  // NOTE: C for some reason does not like 2 operations in one, so for instance
  // uint16_t uci_ver = (rcvd_packet.payload[offset++] << 8) | rcvd_packet.payload[offset++];
  // is undefined behaviour. The existing solution is, IMO, much uglier,
  // but does avoid the warnings so whatever.

  uint8_t status = rcvd_packet.payload[offset++];

  uint16_t uci_ver = rcvd_packet.payload[offset] | (rcvd_packet.payload[offset + 1] << 8);
  offset += 2;

  uint16_t mac_ver = rcvd_packet.payload[offset] | (rcvd_packet.payload[offset + 1] << 8);
  offset += 2;

  uint16_t phy_ver = rcvd_packet.payload[offset] | (rcvd_packet.payload[offset + 1] << 8);
  offset += 2;

  uint16_t test_ver = rcvd_packet.payload[offset] | (rcvd_packet.payload[offset + 1] << 8);
  offset += 2;

  uint8_t v_info_len = rcvd_packet.payload[offset++];

  uint32_t device_id = rcvd_packet.payload[offset] | (rcvd_packet.payload[offset + 1] << 8) |
                       (rcvd_packet.payload[offset + 2] << 16) | (rcvd_packet.payload[offset + 3] << 24);
  offset += 4;

  uint32_t part_id = rcvd_packet.payload[offset] | (rcvd_packet.payload[offset + 1] << 8) |
                     (rcvd_packet.payload[offset + 2] << 16) | (rcvd_packet.payload[offset + 3] << 24);
  offset += 4;

  uint32_t firmware_ver = rcvd_packet.payload[offset] | (rcvd_packet.payload[offset + 1] << 8) |
                          (rcvd_packet.payload[offset + 2] << 16);
  offset += 3;

  uint8_t aoa_cap = rcvd_packet.payload[offset++];

  // print all info
  printf("  cmd: UCI Ver: 0x%04X (%d)\n", uci_ver, uci_ver);
  printf("  cmd: MAC Ver: 0x%04X (%d)\n", mac_ver, mac_ver);
  printf("  cmd: Physical Ver: 0x%04X (%d)\n", phy_ver, phy_ver);
  printf("  cmd: Test Ver: 0x%04X (%d)\n", test_ver, test_ver);
  printf("  cmd: Device ID: 0x%08X (%d)\n", device_id, device_id);
  printf("  cmd: Part ID: 0x%08X (%d)\n", part_id, part_id);
  printf("  cmd: Firmware Ver: 0x%06X (%d)\n", firmware_ver, firmware_ver);
  printf("  cmd: AOA Capability: %s\n", aoa_cap == 0 ? "Error" : aoa_cap == 1 ? "True"
                                                                              : "False");

  return 1; // Success
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
  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1; // Failed to send command
  }

  // Receive the response
  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != SESSION_INIT)
  {
    printf("  cmd: Unexpected response received: GID: 0x%02x, OID: 0x%02x\n", rcvd_packet.gid, rcvd_packet.oid);
    return -2; // Failed to receive valid response
  }

  if (rcvd_packet.payload_len > 0 && rcvd_packet.payload[0] == 0x03)
  {
    printf("  cmd: Received Syntax Error response\n");
    return -3; // Syntax error
  }

  return 1; // Success
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

  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != SESSION_SET_APP_CONFIG)
  {
    return -2;
  }

  return 1;
}

int start_uwb_ranging_session(uint32_t session_id)
{
  uint8_t payload[4];
  payload[0] = (uint8_t)(session_id & 0xFF);
  payload[1] = (uint8_t)((session_id >> 8) & 0xFF);
  payload[2] = (uint8_t)((session_id >> 16) & 0xFF);
  payload[3] = (uint8_t)((session_id >> 24) & 0xFF);

  ControlPacket packet = create_packet(COMMAND, COMPLETE, RANGING, RANGE_START, payload, 4);

  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != RANGE_START)
  {
    return -2;
  }

  return 1;
}

int stop_uwb_ranging_session(uint32_t session_id)
{
  uint8_t payload[4];
  payload[0] = (uint8_t)(session_id & 0xFF);
  payload[1] = (uint8_t)((session_id >> 8) & 0xFF);
  payload[2] = (uint8_t)((session_id >> 16) & 0xFF);
  payload[3] = (uint8_t)((session_id >> 24) & 0xFF);

  ControlPacket packet = create_packet(COMMAND, COMPLETE, RANGING, RANGE_STOP, payload, 4);

  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != RANGE_STOP)
  {
    return -2;
  }

  return 1;
}
