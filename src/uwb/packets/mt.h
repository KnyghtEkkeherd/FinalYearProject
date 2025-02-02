#ifndef MT_H_
#define MT_H_

// Message Type (MT) Definitions
typedef enum
{
  MT_RFU_START = 0b000,    // Reserved for Future Use (RFU)
  MT_COMMAND = 0b001,      // Control Packet - Command message as Information
  MT_RESPONSE = 0b010,     // Control Packet - Response message as Information
  MT_NOTIFICATION = 0b011, // Control Packet - Notification message as Information
  MT_RFU_STOP = 0b100      // Reserved for Future Use (0b100 - 0b111)
} MessageType;

// Function to check if a given MessageType is valid
static inline int is_valid_message_type(MessageType mt)
{
  return (mt == MT_COMMAND || mt == MT_RESPONSE || mt == MT_NOTIFICATION);
}

#endif // MT_H_