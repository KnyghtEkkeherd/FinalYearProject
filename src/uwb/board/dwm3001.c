#include "dwm3001.h"

// Placeholder for UART or USB read/write functions
extern int transport_send(const uint8_t *data, size_t len);
extern int transport_receive(uint8_t *buffer, size_t len);

// Function to send a command to the DWM3001 device
int send_cmd(uint8_t cmd, uint8_t cmd_grp, const uint8_t *data, size_t len)
{
  if (tty_fd == -1) // Ensure TTY port is initialized
  {
    fprintf(stderr, "Error: TTY port not initialized.\n");
    return -1;
  }

  ControlPacket packet;
  // Initialize the control packet
  init_control_packet(&packet, MT_COMMAND, PBF_COMPLETE_OR_LAST_SEGMENT, cmd_grp, cmd, len, data);

  // Check if the packet is valid
  if (!is_valid_control_packet(&packet))
  {
    fprintf(stderr, "Error: Invalid packet.\n");
    return -2; // Invalid packet error code
  }

  // Send the packet
  int result = transport_send((uint8_t *)&packet, 4 + packet.payload_length);
  if (result < 0)
  {
    fprintf(stderr, "Error: Failed to send command.\n");
    return -3; // Transmission error code
  }

  printf("Command sent: cmd=%d, grp=%d, len=%zu\n", cmd, cmd_grp, len);
  return 0; // Success
}

// Device initialization function
int init(void)
{
  printf("Initializing device...\n");
  // Initialize TTY port (example)
  if (tty_init("/dev/tty.usbmodemD02FE9942ABC1", 115200) < 0)
  {
    fprintf(stderr, "Error: Failed to initialize TTY port.\n");
    return -1;
  }
  printf("TTY port initialized successfully.\n");

  // Send a reset command to the device
  int result = send_cmd(CORE_DEVICE_RESET, GID_UCI_CORE, NULL, 0);
  if (result < 0)
  {
    fprintf(stderr, "Error: Failed to send reset command.\n");
    return -1;
  }

  return 0;
}

// Start ranging session
int start_ranging(void)
{
  printf("Starting ranging session...\n");
  int res = send_cmd(RANGE_START, GID_UWB_RANGING, NULL, 0);
  if (res < 0)
  {
    fprintf(stderr, "Error: Failed to start ranging session.\n");
    return -1;
  }

  return 0;
}

// Stop ranging session
int stop_ranging(void)
{
  printf("Stopping ranging session...\n");
  int res = send_cmd(RANGE_STOP, GID_UWB_RANGING, NULL, 0);
  if (res < 0)
  {
    fprintf(stderr, "Error: Failed to stop ranging session.\n");
    return -1;
  }

  return 0;
}

// Get device information
int get_device_info(void)
{
  printf("Getting device information...\n");
  // Send the get device info command
  int result = send_cmd(CORE_GET_DEVICE_INFO, GID_UCI_CORE, NULL, 0);
  if (result < 0)
  {
    fprintf(stderr, "Error: Failed to send get device info command.\n");
    return -1;
  }

  return 0;
}

// Receive data
int rcv_data(uint8_t *buffer, size_t len)
{
  if (tty_fd == -1) // Ensure TTY port is initialized
  {
    fprintf(stderr, "Error: TTY port not initialized.\n");
    return -1;
  }

  printf("Receiving data...\n");
  int bytes_read = transport_receive(buffer, len);
  if (bytes_read > 0)
  {
    printf("Received data: ");
    for (int i = 0; i < bytes_read; i++)
    {
      printf("%02X ", buffer[i]);
    }
    printf("\n");
  }

  return bytes_read;
}

int proc_resp(const uint8_t *response, size_t len)
{
  if (len < 4)
    return -1; // Not enough data error

  // Parse the control packet
  ControlPacket packet;
  parse_control_packet(&packet, response, len);

  // Check if the packet is valid
  if (!is_valid_control_packet(&packet))
  {
    fprintf(stderr, "Error: Invalid packet.\n");
    return -2; // Invalid packet error code
  }

  // Print the packet information
  print_control_packet(&packet);

  return -1; // Unknown message type or error status
}