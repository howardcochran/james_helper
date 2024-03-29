#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include "button.h"
#include "delay.h"
#include "pins.h"

Button::Button(RawButton& raw_button, QueueHandle_t output_queue)
  : raw_button_(raw_button),
    output_queue_(output_queue)
{
}

void Button::task()
{
  char out_buf[64];
  int prevButtonState = UP;
  TickType_t prevStateChangeTime;

  while (true)
  {
    TickType_t curTime = xTaskGetTickCount();
    int curButtonState = raw_button_.getState();
    if (curButtonState != prevButtonState)
    {
      ButtonEvent event;
      TickType_t elapsed = curTime - prevStateChangeTime;
      event.state = curButtonState;
      event.duration = elapsed;
      xQueueSend(output_queue_, &event, 0);

      if (curButtonState == DOWN)
      {
        tone(PIN_BUZZER, 440, 50);
      }
      else
      {
        tone(PIN_BUZZER, 880, 50);
      }
      prevStateChangeTime = curTime;
      prevButtonState = curButtonState;
    }
    taskDelayMsUntil(&curTime, 10);
  }

  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}

void Button::taskEntry(void *instance)
{
  (static_cast<Button*>(instance))->task();
}
