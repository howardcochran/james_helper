#define USE_USBCON
#include <FreeRTOS_SAMD21.h>
#include "james_helper.h"
#include <queue.h>
#include "debug.h"
#include "button.h"
#include "gpio_button.h"
#include "delay.h"
#include "morse.h"
#include "nurse_call.h"

App james_helper;

void App::init()
{

  Serial.begin(115200);
  vNopDelayMS(3000); // prevents usb driver crash on startup, do not omit this
  while (!Serial)
    ; // empty

  debug_init(nh_);
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object.
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack!
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  QueueHandle_t trigger_queue = xQueueCreate(20, sizeof(ButtonEvent));
  // GPIOButton raw_trigger(MAIN_BUTTON_PIN);
  prox_button_.init(trigger_queue);
  Button main_trigger(prox_button_, trigger_queue);

  TaskHandle_t TaskHandle_buttonRead;
  xTaskCreate(Button::taskEntry, "buttonRead", 256, &main_trigger, tskIDLE_PRIORITY + 3, &TaskHandle_buttonRead);

  nurse_caller_.init(trigger_queue, 12);
  morse_decoder_.init(trigger_queue);
  hardware_button_.init(trigger_queue, 12);

  memset(modes_, 0, sizeof(modes_));
  modes_[(int)AppMode::NURSE_CALL] = &nurse_caller_;
  modes_[(int)AppMode::MORSE_KEYBOARD] = &morse_decoder_;
  modes_[(int)AppMode::HARDWARE_BUTTON] = &hardware_button_;
  //modes_[(int)AppMode::BT_BUTTON] = ;
  //modes_[(int)AppMode::MENU] = ;
  james_helper.set_major_mode(App::AppMode::NURSE_CALL);
  vTaskStartScheduler();
}

void setup()
{
  james_helper.init();
}

void App::suspend_all_modes(void)
{
  for (int i = 0; i < (int)AppMode::MODE_COUNT; i++)
  {
    if (modes_[i])
      modes_[i]->suspend();
  }
}

void App::set_major_mode(AppMode mode)
{
  debug("set_major_mode %d\n", (int)mode);
  cur_mode_ = mode;
  suspend_all_modes();
  modes_[(int)mode]->resume();
}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop()
{
  james_helper.getNodeHandle()->spinOnce();
  vNopDelayMS(1);
}
