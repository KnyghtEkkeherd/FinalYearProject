#include "transport/tty.h"
#include "packet/commands.h"
#include <unistd.h>

// Define the TTY device path and baud rate
#define TTY_PATH "/dev/tty.usbmodemCC210983BA011"
#define BAUD_RATE B115200

// Define session vars
#define SESSION_ID 42
#define SESSION_TYPE 0

int main()
{

  // Initialize the TTY port
  if (tty_init(TTY_PATH, BAUD_RATE) != 0)
  {
    return -1; // Initialization failed
  }

  // get_device_info(); // Use this to test, should get 0xDECA0302 as device id
  // return 0;
  printf("\n -- Resetting UWB... -- \n");
  if (reset_device() < 0)
  {
    printf("\n**ERROR: FAILED TO RESET DEVICE**\n");
    gtfo(SESSION_ID);
    return -1;
  }
  sleep(3);

  printf("\n -- Resetting TTY... -- \n");
  if (tty_init(TTY_PATH, BAUD_RATE) != 0)
  {
    gtfo(SESSION_ID);
    return -2; // Initialization failed
  }

  printf("\n -- Initializing UWB session... -- \n");
  if (init_uwb_session(SESSION_ID, SESSION_TYPE) < 0)
  {
    printf("\n**ERROR: FAILED TO INIT UWB SESSION**\n");
    gtfo(SESSION_ID);
    return -3;
  }
  sleep(3);

  printf("\n -- Setting session parameters... -- \n");
  if (set_uwb_session_parameters(SESSION_ID, SESSION_TYPE) < 0)
  {
    printf("\n**ERROR: FAILED TO SET SESSION PARAMS**\n");
    gtfo(SESSION_ID);
    return -4;
  }
  sleep(3);

  printf("\n -- Starting UWB ranging... -- \n");
  if (start_uwb_ranging_session(SESSION_ID) < 0)
  {
    printf("\n**ERROR: FAILED TO START UWB RANGING**\n");
    gtfo(SESSION_ID);
    return -5;
  }
  sleep(3);

  gtfo(SESSION_ID);

  // Close the TTY port
  tty_close();
  return 1; // Success
}
