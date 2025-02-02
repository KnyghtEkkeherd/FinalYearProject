#ifndef TTY_H_
#define TTY_H_

#include <stdint.h>
#include <fcntl.h>   // For open()
#include <unistd.h>  // For close(), read(), write()
#include <termios.h> // For termios
#include <stdio.h>   // For printf()

static int tty_fd = -1; // File descriptor for the TTY

// Function to initialize the TTY port
int tty_init(const char *tty_path, uint32_t baud_rate);

// Function to send data over the TTY port
int transport_send(const uint8_t *data, size_t len);

// Function to receive data over the TTY port
int transport_receive(uint8_t *buffer, size_t len);

// Function to continuously read and print incoming data
void read_and_print_data();

// Function to close the TTY port
void tty_close();

#endif // TTY_H_
