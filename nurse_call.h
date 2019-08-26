#ifndef __JAMES_HELPER__NURSE_CALL_H
#define __JAMES_HELPER__NURSE_CALL_H
#include "major_mode.h"
#include "button.h"

class NurseCall : public MajorMode
{
public:
  void init(QueueHandle_t input_queue);
  void callNurse(void);
  void task(void);

protected:
  QueueHandle_t input_queue_;
  const int time_limit_ = 5000;
  const int clicks_to_trigger_ = 5;
  const int relay_pin_ = 12;
  const int buzzer_pin_ = 14;
};

#endif
