#ifndef DEVICE_CONFIG_H_
#define DEVICE_CONFIG_H_

// Device configuration parameters
#define DEVICE_STATE 0x00   // State reported by CORE_DEVICE_STATUS, read only
#define LOW_POWER_MODE 0x01 // Enable (1) or disable (0) low power mode, see FiRa UCI specification
#define CHANNEL 0xa0        // The device default channel
#define PREAMBLE_CODE 0xa1  // The device default preamble code
#define PAN_ID 0xa2         // The identifier of the PAN on which the device is operating, if 0xffff, device is not associated
#define SHORT_ADDR 0xa3     // The address the device uses to communicate in the PAN
#define EXTENDED_ADDR 0xa4  // The extended address assigned to the device
#define PAN_COORD 0xa5      // Device is a PAN coordinator if 1
#define PROMISCUOUS 0xa6    // Enable (1) or disable (0) promiscuous mode
#define FRAME_RETRIES 0xa7  // The maximum number of retries after a transmission failure

#endif // DEVICE_CONFIG_H_
