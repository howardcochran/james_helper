#include <stdarg.h>
#include <FreeRTOS_SAMD21.h>
#include <Arduino.h>
#include <semphr.h>
#include <ros.h>
#include "delay.h"
#include "debug.h"

static char buf[256];
static char isr_buf[64];
static bool is_inited;
static SemaphoreHandle_t debug_print_mutex;
static ros::NodeHandle *nh;

void debug_init(ros::NodeHandle &node_handle)
{
  nh = &node_handle;
  debug_print_mutex = xSemaphoreCreateMutex();
  is_inited = true;
}

int debug(const char *fmt, ...)
{
  xSemaphoreTake(debug_print_mutex, portMAX_DELAY);
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  //Serial.print(buf);
  if (len && buf[len - 1] == '\n')  // strip superfluous newline
  {
    buf[len - 1] = '\0';
    --len;
  }
  nh->loginfo(buf);
  xSemaphoreGive(debug_print_mutex);
  return len;
}

int debugFromISR(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  int count = vsnprintf(isr_buf, sizeof(isr_buf), fmt, args);
  va_end(args);
  //Serial.print(isr_buf);
  return count;
}
