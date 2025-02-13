#include "transport/tty.h"
#include "packet/commands.h"

// Define the TTY device path and baud rate
#define TTY_PATH "/dev/tty.usbmodemCC210983BA011"
#define BAUD_RATE B115200

int main()
{

  // Initialize the TTY port
  if (tty_init(TTY_PATH, BAUD_RATE) != 0)
  {
    return -1; // Initialization failed
  }

  // get_device_info(); // Use this to test
  // reset_device();
  // init_uwb_session(42, 0);
  // set_uwb_session_parameters(42);
  // start_uwb_ranging_session(42);
  stop_uwb_ranging_session(42);

  // Close the TTY port
  tty_close();
  return 0; // Success
}
