#include "tty.h"

// Function to initialize the TTY port
int tty_init(const char *tty_path, uint32_t baud_rate)
{
  struct termios options;

  // Open the TTY port
  tty_fd = open(tty_path, O_RDWR | O_NOCTTY | O_NDELAY);
  if (tty_fd == -1)
  {
    perror("Unable to open TTY");
    return -1;
  }

  // Get the current options for the port
  tcgetattr(tty_fd, &options);

  // Set the baud rate
  cfsetispeed(&options, baud_rate); // Input baud rate
  cfsetospeed(&options, baud_rate); // Output baud rate

  // 8N1: 8 data bits, no parity, 1 stop bit
  options.c_cflag &= ~PARENB; // No parity
  options.c_cflag &= ~CSTOPB; // 1 stop bit
  options.c_cflag &= ~CSIZE;  // Clear the current data size setting
  options.c_cflag |= CS8;     // 8 data bits

  // Enable the receiver and set local mode
  options.c_cflag |= (CLOCAL | CREAD);

  // Apply the settings
  tcsetattr(tty_fd, TCSANOW, &options);

  // Set the read timeout
  options.c_cc[VMIN] = 1;   // Minimum number of characters to read
  options.c_cc[VTIME] = 20; // Timeout in tenths of seconds
  tcsetattr(tty_fd, TCSANOW, &options);

  // Set the port to blocking mode
  fcntl(tty_fd, F_SETFL, 0);

  return 0; // Success
}

// Function to send data over the TTY port
int transport_send(const uint8_t *data, size_t len)
{
  if (tty_fd == -1)
  {
    perror("TTY not initialized");
    return -1;
  }

  ssize_t bytes_written = write(tty_fd, data, len);
  if (bytes_written < 0)
  {
    perror("Failed to write to TTY");
    return -1;
  }

  printf("Sent %ld bytes\n", bytes_written);
  return bytes_written; // Return number of bytes sent
}

// Function to receive data over the TTY port
int transport_receive(uint8_t *buffer, size_t len)
{
  if (tty_fd == -1)
  {
    perror("TTY not initialized");
    return -1;
  }

  ssize_t bytes_read = read(tty_fd, buffer, len);
  if (bytes_read < 0)
  {
    perror("Failed to read from TTY");
    return -1;
  }

  printf("Received %ld bytes\n", bytes_read);
  return bytes_read; // Return number of bytes received
}

// Function to continuously read and print incoming data
void read_and_print_data()
{
  uint8_t buffer[256];
  while (1)
  {
    int bytes_read = transport_receive(buffer, sizeof(buffer));
    if (bytes_read > 0)
    {
      // Print the received data
      for (int i = 0; i < bytes_read; i++)
      {
        printf("Data: %02X\n", buffer[i]);
      }
      printf("\n");
    }
  }
}

// Function to close the TTY port
void tty_close()
{
  if (tty_fd != -1)
  {
    close(tty_fd);
    tty_fd = -1;
  }
}