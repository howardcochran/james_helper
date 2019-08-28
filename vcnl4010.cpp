#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include "delay.h"
#include "debug.h"
#include "vcnl4010.h"

void Vcnl4010::init(QueueHandle_t output_queue)
{
  output_queue_ = output_queue;
  if (!vcnl_.begin())
  {
    debug("Proximity sensor VNCL4010 not found!\n");
  }
  create_task("VNCL4010_driver");
}

void Vcnl4010::task()
{
  TickType_t start_time = xTaskGetTickCount() - 1;
  int count = 0;
  while (true)
  {
    int cur_prox = vcnl_.readProximity();
    TickType_t cur_time = xTaskGetTickCount();
    ++count;
    int rate = count * 1000 / (cur_time - start_time);
    float inv = 1000.0 * 65536 / cur_prox;
    debug("prox: %d inv: %d rate: %d\n", cur_prox, (int)inv, rate);
    taskDelayMs(1);
  }
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}
