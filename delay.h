#ifndef __JAMES_HELPER__DELAY_H
#define __JAMES_HELPER__DELAY_H
inline void taskDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );
}

inline void taskDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );
}

inline void taskDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );
}

#endif
