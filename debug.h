#ifndef __JAMES_HELPER__DEBUG_H
#define __JAMES_HELPER__DEBUG_H
#include <ros.h>
#include "delay.h"

#define DELAY_MS 500

void debug_init(ros::NodeHandle &node_handle);
int debug(const char *fmt, ...);
int debugFromISR(const char *fmt, ...);

#define debug_wait(FMT, ...) \
    do { \
      debug(FMT, ##__VA_ARGS__); \
      taskDelayMs(DELAY_MS); \
    } while(false)

#define debug_earlywait(FMT, ...) \
    do { \
      debug(FMT, ##__VA_ARGS__); \
      vNopDelayMS(DELAY_MS); \
    } while(false)

#endif
