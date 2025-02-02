#ifndef PBF_H_
#define PBF_H_

// Packet Boundary Flag (PBF) Definitions
typedef enum
{
  PBF_COMPLETE_OR_LAST_SEGMENT = 0b0, // Complete message or last segment of a segmented message
  PBF_NOT_LAST_SEGMENT = 0b1          // Segment of a message that is not the last segment
} PacketBoundaryFlag;

// Function to check if a given PBF value is valid
static inline int is_valid_pbf(PacketBoundaryFlag pbf)
{
  return (pbf == PBF_COMPLETE_OR_LAST_SEGMENT || pbf == PBF_NOT_LAST_SEGMENT);
}

#endif // PBF_H_