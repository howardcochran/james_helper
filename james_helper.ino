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
#include "pins.h"

App james_helper;
void modeSwitchThread(void *unused);

void App::init()
{
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  tone(PIN_BUZZER, 880, 200);
  vNopDelayMS(2000);
  tone(PIN_BUZZER, 700, 101);
  vNopDelayMS(100);
  tone(PIN_BUZZER, 880, 100);

  nh_.getHardware()->setBaud(115200);
  nh_.initNode();
  debug_init(nh_);
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object.
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack!
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(PIN_LED_BUILTIN, LOW);

  QueueHandle_t trigger_queue = xQueueCreate(20, sizeof(ButtonEvent));
  // GPIOButton raw_trigger(PIN_RED_BUTTON);
  prox_button_.init(trigger_queue, nh_);
  Button main_trigger(prox_button_, trigger_queue);

  TaskHandle_t TaskHandle_buttonRead;
  xTaskCreate(Button::taskEntry, "buttonRead", 256, &main_trigger, tskIDLE_PRIORITY + 3, &TaskHandle_buttonRead);

  nurse_caller_.init(trigger_queue, PIN_RELAY);
  morse_decoder_.init(trigger_queue);
  hardware_button_.init(trigger_queue, PIN_RELAY);

  memset(modes_, 0, sizeof(modes_));
  modes_[(int)AppMode::NURSE_CALL] = &nurse_caller_;
  modes_[(int)AppMode::MORSE_KEYBOARD] = &morse_decoder_;
  modes_[(int)AppMode::HARDWARE_BUTTON] = &hardware_button_;
  //modes_[(int)AppMode::BT_BUTTON] = ;
  //modes_[(int)AppMode::MENU] = ;
  james_helper.setMajorMode(App::AppMode::NURSE_CALL);

  TaskHandle_t TaskHandle_modeSwitch;
  xTaskCreate(modeSwitchThread, "modeSwitch", 256, NULL, tskIDLE_PRIORITY + 4, &TaskHandle_modeSwitch);
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

void App::setMajorMode(AppMode mode)
{
  debug("setMajorMode %d\n", (int)mode);
  cur_mode_ = mode;
  suspend_all_modes();
  modes_[(int)mode]->resume();
}

void modeSwitchThread(void *unused)
{
  pinMode(PIN_MID_TOGGLE_SWITCH, INPUT_PULLUP);
  while (true)
  {
    vTaskDelay(500);
    int mode_switch_pos = digitalRead(PIN_MID_TOGGLE_SWITCH);  // LOW = Nurse Call, HIGH = HW button
    App::AppMode desired_mode;
    if (mode_switch_pos == HIGH)
      desired_mode = App::AppMode::NURSE_CALL;
    else
      desired_mode = App::AppMode::HARDWARE_BUTTON;
    App::AppMode cur_mode = james_helper.getMajorMode();
    if (cur_mode != desired_mode)
    {
      debug("ModeSwitch: Changimg mode to: %d\n", desired_mode);
      james_helper.setMajorMode(desired_mode);
    }
  }
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
