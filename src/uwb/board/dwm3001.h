#ifndef DWM3001_H_
#define DWM3001_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "../packets/headers/cp.h"
#include "../packets/headers/cph.h"
#include "../transport/tty.c"
#include "../config/messages_id.h"

// Placeholder for UART or USB read/write functions
extern int transport_send(const uint8_t *data, size_t len);
extern int transport_receive(uint8_t *buffer, size_t len);

// Function to send a command to the DWM3001 device
int send_cmd(uint8_t cmd, uint8_t cmd_grp, const uint8_t *data, size_t len);

// Device initialization function
int init(void);

// Start ranging session
int start_ranging(void);

// Stop ranging session
int stop_ranging(void);

// Receive data
int rcv_data(uint8_t *buffer, size_t len);

// Process response
int proc_resp(const uint8_t *response, size_t len);

// Get device information
int proc_device_info_resp(const uint8_t *response, size_t len);

#endif // DWM3001_H_
