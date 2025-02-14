#ifndef HEADER_H
#define HEADER_H

#include "fields/mt.h"
#include "fields/pbf.h"

typedef struct
{
  MessageType mt;
  PacketBoundaryFlag pbf;
} PacketHeader;

#endif // HEADER_H