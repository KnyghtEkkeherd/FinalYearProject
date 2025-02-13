#ifndef PBF_H
#define PBF_H

typedef enum
{
  COMPLETE = 0b0,
  INCOMPLETE = 0b1
} PacketBoundaryFlag;

const char *pbf_t_s(PacketBoundaryFlag pbf)
{
  switch (pbf)
  {
  case COMPLETE:
    return "COMPLETE";
  case INCOMPLETE:
    return "INCOMPLETE";
  default:
    return "UNKNOWN";
  }
}

#endif // PBF_H