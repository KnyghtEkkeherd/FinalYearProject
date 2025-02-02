#include "board/dwm3001.c"

int main(void)
{
  if (init() < 0)
  {
    fprintf(stderr, "Error: Failed to initialize device.\n");
    return -1;
  }
  get_device_info();

  // Receive and process the response
  uint8_t buffer[256];
  int bytes_read = rcv_data(buffer, sizeof(buffer));

  // Process the response
  if (bytes_read < 0)
  {
    fprintf(stderr, "Error: Failed to receive data.\n");
    return -1;
  }

  int res = proc_resp(buffer, bytes_read);
  if (res < 0)
  {
    fprintf(stderr, "Error: Failed to process device info response.\n");
    return -1;
  }

  return 0;
}
