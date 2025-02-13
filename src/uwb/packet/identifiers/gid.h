#ifndef GID_H
#define GID_H

typedef enum
{
  CORE = 0x0,
  SESSION = 0x1,
  RANGING = 0x2,
} GroupIdentifier;

const char *gid_t_s(GroupIdentifier gid)
{
  switch (gid)
  {
  case CORE:
    return "CORE";
  case SESSION:
    return "SESSION";
  case RANGING:
    return "RANGING";
  default:
    return "UNKNOWN";
  }
}

#endif // GID_H