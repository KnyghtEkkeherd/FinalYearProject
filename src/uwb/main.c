#include "transport/tty.h"
#include "packet/controlPacket.h"

// Define the TTY device path and baud rate
#define TTY_PATH "/dev/tty.usbmodemCC210983BA011"
#define BAUD_RATE B115200

int main()
{
  uint8_t buffer[MAX_PAYLOAD_SIZE + 4];
  uint8_t payload[MAX_PAYLOAD_SIZE];
  uint8_t payload_len;

  // Initialize the TTY port
  if (tty_init(TTY_PATH, BAUD_RATE) != 0)
  {
    return -1; // Initialization failed
  }

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

  // Close the TTY port
  tty_close();
  return 0; // Success
}
