#ifndef __JAMES_HELPER__DEBUG_H
#define __JAMES_HELPER__DEBUG_H
#include "delay.h"

#define DELAY_MS 500

void debug_init(void);
int debug(const char *fmt, ...);
//int debug_wait(const char *fmt, ...);
//int debug_earlywait(const char *fmt, ...);
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
