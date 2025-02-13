#include "fields/mt.h"
#include "fields/pbf.h"

typedef struct
{
  MessageType mt;
  PacketBoundaryFlag pbf;
} PacketHeader;