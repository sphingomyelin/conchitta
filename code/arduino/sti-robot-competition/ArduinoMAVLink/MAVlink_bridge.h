#ifndef MAVlink_bridge.h
#define MAVlink_bridge.h

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include <mavlink_types.h>


mavlink_system_t mavlink_system;
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

void comm_receive();

#endif
