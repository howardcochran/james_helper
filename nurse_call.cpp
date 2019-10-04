#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include "button.h"
#include "delay.h"
#include "debug.h"
#include "nurse_call.h"
#include "note_pitches.h"
#include "pins.h"

void NurseCall::init(QueueHandle_t input_queue, int relay_pin)
{
  input_queue_ = input_queue;
  relay_pin_ = relay_pin;
  digitalWrite(relay_pin_, LOW);
  pinMode(relay_pin_, OUTPUT);
  digitalWrite(PIN_LED_RED, LOW);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_RED_BUTTON, INPUT);
  create_task("nurse call");
}

void NurseCall::callNurse(void) {
  is_call_active_ = true;
  int note_dur = 100;
  digitalWrite(relay_pin_, HIGH);
  digitalWrite(PIN_LED_RED, HIGH);
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
  digitalWrite(PIN_LED_RED, LOW);
}

void NurseCall::clearNurseCall()
{
  int note_dur = 200;
  debug ("Claring Nurse Call state");
  is_call_active_ = false;
  tone(buzzer_pin_, NOTE_G4);
  taskDelayMs(note_dur);
  tone(buzzer_pin_, NOTE_G3);
  taskDelayMs(note_dur);
  noTone(buzzer_pin_);
  digitalWrite(relay_pin_, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  is_led_on_ = false;
}

void NurseCall::updateUI()
{
  if (is_call_active_)
  {
    if (digitalRead(PIN_RED_BUTTON) == HIGH)
    {
      clearNurseCall();
      return;
    }
    is_led_on_ = !is_led_on_;
  }
  else
  {
    is_led_on_ = false;
  }
  digitalWrite(PIN_LED_RED, is_led_on_);
}

void NurseCall::task(void)
{
  ButtonEvent event = {UP, 0};
  TickType_t start_time = 0;
  int cur_clicks = 0;

  while(true)
  {
    bool have_event = xQueueReceive(input_queue_, &event, 100 / portTICK_PERIOD_MS);
    updateUI();
    if (!have_event)
    {
      continue;
    }
    else if (event.state == UP)
    {
      tone(buzzer_pin_, 440, 50);
      continue;
    }
    else
    {
      tone(buzzer_pin_, 220, 50);
      debug("event.state: %d, clicks: %d\n", event.state, cur_clicks);
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
          start_time = 0;
          cur_clicks = 0;
          if (is_call_active_)
            clearNurseCall();
          else
            callNurse();
        }
      }
    }
  }
}

void NurseCall::suspend(void)
{
  BaseTask::suspend();
  int note_dur = 200;
  is_call_active_ = false;
  is_led_on_ = false;
  noTone(buzzer_pin_);
  digitalWrite(relay_pin_, LOW);
  digitalWrite(PIN_LED_RED, LOW);
}
