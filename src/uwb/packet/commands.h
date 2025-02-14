#include "../transport/tty.h"
#include "controlPacket.h"
#include "notifPacket.h"

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
  if (rcvd_packet.gid != CORE || rcvd_packet.oid != CORE_DEVICE_RESET || rcvd_packet.payload[0] != STATUS_OK)
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
  if (rcvd_packet.gid) // if its 0
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

  sleep(1);

  // Receive the response
  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != SESSION_INIT || rcvd_packet.payload[0] != STATUS_OK)
  {
    return -2; // Failed to receive valid response
  }

  return 1; // Success
}

int set_uwb_session_parameters(uint32_t sid, uint8_t stype)
{
  uint8_t session_params[] = {
      0x00, 0x01,       // DeviceType: Controller (0x00), DeviceRole: Initiator (session_type)
      0x06, 0x00, 0x01, // DeviceMacAddress: 0x00000001
      0x05, 0x01,       // NumberOfControlees: 1
      0x07, 0x00, 0x02, // DstMacAddress: 0x00000002
      0x04, 0x09,       // ChannelNumber: Channel 9
  };
  uint8_t session_params_len = sizeof(session_params);

  uint8_t payload[MAX_PAYLOAD_SIZE];
  int offset = 0;

  payload[offset++] = (uint8_t)(sid & 0xFF);
  payload[offset++] = (uint8_t)((sid >> 8) & 0xFF);
  payload[offset++] = (uint8_t)((sid >> 16) & 0xFF);
  payload[offset++] = (uint8_t)((sid >> 24) & 0xFF);

  for (int i = 0; i < session_params_len; i++)
  {
    payload[offset++] = session_params[i];
  }

  ControlPacket packet = create_packet(COMMAND, COMPLETE, SESSION, SESSION_SET_APP_CONFIG, payload, offset);
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != SESSION_SET_APP_CONFIG || rcvd_packet.payload[0] != STATUS_OK)
  {
    return -2;
  }

  return 1;
}

int get_uwb_session_parameters(uint32_t sid)
{
  uint8_t payload[4]; // 4 bytes for session ID
  int offset = 0;

  // Encode the session ID
  payload[offset++] = (uint8_t)(sid & 0xFF);
  payload[offset++] = (uint8_t)((sid >> 8) & 0xFF);
  payload[offset++] = (uint8_t)((sid >> 16) & 0xFF);
  payload[offset++] = (uint8_t)((sid >> 24) & 0xFF);

  // Create the packet
  ControlPacket packet = create_packet(COMMAND, COMPLETE, SESSION, SESSION_GET_APP_CONFIG, payload, offset);

  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    printf("  cmd: Failed to send SESSION_GET_APP_CONFIG command.\n");
    return -1; // Failed to send command
  }

  // Receive the response
  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != SESSION_GET_APP_CONFIG)
  {
    printf("  cmd: Unexpected response received: GID: 0x%02x, OID: 0x%02x\n", rcvd_packet.gid, rcvd_packet.oid);
    return -2; // Failed to receive valid response
  }

  // Parse the received payload
  uint8_t status = rcvd_packet.payload[0];
  if (status != 0x00)
  {
    printf("  cmd: Error retrieving session configuration, status: 0x%02x\n", status);
    return -3; // Error status received
  }

  // Process the TLV data (Type-Length-Value)
  uint8_t *tlv_payload = rcvd_packet.payload + 1;
  int tlv_length = rcvd_packet.payload_len - 1; // Adjust based on actual payload length

  // Example parsing loop
  while (tlv_length > 0)
  {
    uint8_t type = *tlv_payload;
    uint8_t length = *(tlv_payload + 1);
    uint8_t *value = tlv_payload + 2;

    // Print or process the TLV
    printf("Type: 0x%02x, Length: %d, Value: ", type, length);
    for (int i = 0; i < length; i++)
    {
      printf("0x%02x ", value[i]);
    }
    printf("\n");

    // Move to the next TLV
    tlv_payload += 2 + length;
    tlv_length -= 2 + length;
  }

  return 1; // Success
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
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != RANGE_START || rcvd_packet.payload[0] != STATUS_OK)
  {
    return -2;
  }

  return 1;
}

int stop_uwb_ranging_session(uint32_t sid)
{
  uint8_t payload[4];
  payload[0] = (uint8_t)(sid & 0xFF);
  payload[1] = (uint8_t)((sid >> 8) & 0xFF);
  payload[2] = (uint8_t)((sid >> 16) & 0xFF);
  payload[3] = (uint8_t)((sid >> 24) & 0xFF);

  ControlPacket packet = create_packet(COMMAND, COMPLETE, RANGING, RANGE_STOP, payload, 4);

  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != RANGE_STOP || rcvd_packet.payload[0] != STATUS_OK)
  {
    return -2;
  }

  return 1;
}

int deinit_uwb_session(uint32_t sid)
{
  uint8_t payload[4];
  payload[0] = (uint8_t)(sid & 0xFF);
  payload[1] = (uint8_t)((sid >> 8) & 0xFF);
  payload[2] = (uint8_t)((sid >> 16) & 0xFF);
  payload[3] = (uint8_t)((sid >> 24) & 0xFF);

  ControlPacket packet = create_packet(COMMAND, COMPLETE, SESSION, SESSION_DEINIT, payload, 4);

  printf("  cmd: Sending %s command...\n", oid_t_s(packet.gid, packet.oid));
  if (send_packet(packet) < 0)
  {
    return -1;
  }

  ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
  if (rcvd_packet.gid != SESSION || rcvd_packet.oid != SESSION_DEINIT || rcvd_packet.payload[0] != STATUS_OK)
  {
    return -2;
  }

  return 1;
}

void receive_process_notif()
{
  while (1) // Loop to continuously receive and process notifications
  {
    ControlPacket rcvd_packet = rcv_packet(buffer, sizeof(buffer));
    if (rcvd_packet.gid != RANGING || rcvd_packet.oid != RANGE_START || rcvd_packet.payload[0] != STATUS_OK)
    {
      printf("  cmd: Unexpected notification received: GID: 0x%02x, OID: 0x%02x\n", rcvd_packet.gid, rcvd_packet.oid);
      continue; // Skip any unrelated notifications
    }

    // Process the payload
    NotificationPacket rangeData = parse_notif(rcvd_packet.payload);

    // Print or handle the received ranging measurements
    for (uint8_t i = 0; i < rangeData.NumberofRangingMeasurements; ++i)
    {
      printf("  cmd: Ranging Measurement %d:\n", i + 1);
      printf("  cmd: MACAddress: 0x%04X\n", rangeData.MACAddress[i]);
      printf("  cmd: Status: %d\n", rangeData.Status[i]);
      printf("  cmd: NLoS: %d\n", rangeData.NLoS[i]);
      printf("  cmd: Distance: %d cm\n", rangeData.Distance[i]);
    }
  }
}

void gtfo(uint32_t sid)
{
  printf("  cmd: OH SHIT OH SHIT OH SHIT\n");
  stop_uwb_ranging_session(sid);
  deinit_uwb_session(sid);
  sleep(1);
}