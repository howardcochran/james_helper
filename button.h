#ifndef __JAMES_HELPER__BUTTON_H
#define __JAMES_HELPER__BUTTON_H
#include "raw_button.h"

#define TRIGGER_BUZZER_PITCH 200

struct ButtonEvent
{
  int state;
  int duration;
};

class Button
{
public:
  Button(RawButton& raw_button, QueueHandle_t output_queue);
  void task(void);
  static void taskEntry(void *instance);

protected:
  RawButton& raw_button_;
  QueueHandle_t output_queue_;
};

#endif
