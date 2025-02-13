#include "transport/tty.h"
#include "packet/controlPacket.h"

// Define the TTY device path and baud rate
#define TTY_PATH "/dev/tty.usbmodemCC210983BA011"
#define BAUD_RATE B115200

#define MAX_PAYLOAD_SIZE 255

int main()
{
  uint8_t buffer[MAX_PAYLOAD_SIZE + 4];

  // Initialize the TTY port
  if (tty_init(TTY_PATH, BAUD_RATE) != 0)
  {
    return -1; // Initialization failed
  }

  // Create the CORE_GET_DEVICE_INFO packet
  // ControlPacket packet = create_packet(0b00100000, 0x02, 0x00, 0x00, NULL, 0); // this WORKS
  ControlPacket packet = create_packet(COMMAND, COMPLETE, CORE, CORE_GET_DEVICE_INFO, NULL, 0);

  // Send the CORE_GET_DEVICE_INFO command
  printf("Sending CORE_GET_DEVICE_INFO command...\n");
  if (send_packet(packet) < 0)
  {
    return -1; // Failed to send command
  }

  // Receive the response
  printf("Receiving response...\n");
  int bytes_read = tty_rcv(buffer, sizeof(buffer));
  if (bytes_read < 4)
  {
    printf("Failed to receive a valid response header\n");
    return -1; // Failed to receive valid response
  }

  // Print the header
  printf("Received Header: ");
  for (int i = 0; i < 4; i++)
  {
    printf("%02X ", buffer[i]);
  }
  printf("\n");

  // Extract payload length from the header
  uint8_t payload_length = buffer[3];

  // Print the payload
  printf("Received Payload: ");
  for (int i = 4; i < 4 + payload_length; i++)
  {
    printf("%02X ", buffer[i]);
  }
  printf("\n");

  // Close the TTY port
  tty_close();
  return 0; // Success
}
