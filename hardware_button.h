#ifndef __JAMES_HELPER__HARDWARE_BUTTON_H
#define __JAMES_HELPER__HARDWARE_BUTTON_H
#include "base_task.h"
#include "button.h"

class HardwareButton : public BaseTask
{
public:
  void init(QueueHandle_t input_queue, int relay_pin);
  void task(void);

protected:
  QueueHandle_t input_queue_;
  int relay_pin_;
};

#endif

