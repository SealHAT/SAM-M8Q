#ifndef UBLOXNAV_H
#define UBLOXNAV_H

#include "ubxmessage.h"
#ifdef __cplusplus
extern "C"
{
#endif

UBXMsgBuffer getNAV_PVT_POLL();

#ifdef __cplusplus
}
#endif

#endif // UBLOXNAV_H
