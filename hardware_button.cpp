#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include "debug.h"
#include "button.h"
#include "hardware_button.h"

void HardwareButton::init(QueueHandle_t input_queue, int relay_pin)
{
  input_queue_ = input_queue;
  relay_pin_ = relay_pin;
  digitalWrite(relay_pin_, LOW);
  pinMode(relay_pin_, OUTPUT);
  create_task("hardware_button");
  debug("setup hw button\n");
}

void HardwareButton::task(void)
{
  ButtonEvent event = {UP, 0};

  debug("begin hw button\n");
  while(true)
  {
    if (!xQueueReceive(input_queue_, &event, 10000))
    {
      if (event.state == DOWN)
      {
        // Prev state was button down but it has been a long time. Release the relay
        event.state = UP;
      }
      else
      {
        continue;
      }
    }
    digitalWrite(relay_pin_, event.state == DOWN ? HIGH : LOW);
  }
}

