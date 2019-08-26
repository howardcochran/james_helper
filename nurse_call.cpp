#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include "button.h"
#include "delay.h"
#include "nurse_call.h"
#include "note_pitches.h"

void NurseCall::init(QueueHandle_t input_queue)
{
  input_queue_ = input_queue;
  digitalWrite(relay_pin_, LOW);
  pinMode(relay_pin_, OUTPUT);
  create_task("nurse_call");
}

void NurseCall::callNurse(void) {
  int note_dur = 100;
  digitalWrite(relay_pin_, HIGH);
  tone(buzzer_pin_, NOTE_G3);
  taskDelayMs(note_dur);
  tone(buzzer_pin_, NOTE_A3);
  taskDelayMs(note_dur);
  tone(buzzer_pin_, NOTE_B3);
  taskDelayMs(note_dur);
  tone(buzzer_pin_, NOTE_C4);
  taskDelayMs(note_dur);
  tone(buzzer_pin_, NOTE_D4);
  taskDelayMs(note_dur);
  tone(buzzer_pin_, NOTE_E4);
  taskDelayMs(note_dur);
  tone(buzzer_pin_, NOTE_FS4);
  taskDelayMs(note_dur);
  tone(buzzer_pin_, NOTE_G4);
  taskDelayMs(note_dur * 2);
  noTone(buzzer_pin_);
  digitalWrite(relay_pin_, LOW);
}

void NurseCall::task(void)
{
  char out_buf[64];
  ButtonEvent event = {UP, 0};
  TickType_t start_time = 0;
  int cur_clicks = 0;

  while(true)
  {
    if (!xQueueReceive(input_queue_, &event, 1000000000000000000))
    {
      continue;
    }
    else if (event.state == UP)
    {
      continue;
    }
    else
    {
      if (start_time == 0 || (xTaskGetTickCount() - start_time) > time_limit_)
      {
        start_time = xTaskGetTickCount();
        cur_clicks = 1;
      }
      else
      {
        cur_clicks++;
        if (cur_clicks >= clicks_to_trigger_)
        {
          callNurse();
          start_time = 0;
          cur_clicks = 0;
        }
      }
    }
  }
}
