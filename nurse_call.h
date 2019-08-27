#ifndef __JAMES_HELPER__NURSE_CALL_H
#define __JAMES_HELPER__NURSE_CALL_H
#include "base_task.h"
#include "button.h"

class NurseCall : public BaseTask
{
public:
  void init(QueueHandle_t input_queue, int relay_pin);
  void callNurse(void);
  void task(void);

protected:
  QueueHandle_t input_queue_;
  int relay_pin_;
  const int time_limit_ = 5000;
  const int clicks_to_trigger_ = 5;
  const int buzzer_pin_ = 14;
};

#endif
