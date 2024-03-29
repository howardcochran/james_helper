#ifndef __JAMES_HELPER__NURSE_CALL_H
#define __JAMES_HELPER__NURSE_CALL_H
#include "base_task.h"
#include "button.h"
#include "pins.h"

class NurseCall : public BaseTask
{
public:
  void init(QueueHandle_t input_queue, int relay_pin);
  void callNurse(void);
  void clearNurseCall(void);
  void task(void);
  void create_taska(char *name);
  virtual void suspend(void);

protected:
  void updateUI();
  QueueHandle_t input_queue_;
  bool is_call_active_;
  bool is_led_on_;
  int relay_pin_;
  const int time_limit_ = 6000;
  const int clicks_to_trigger_ = 4;
  const int buzzer_pin_ = PIN_BUZZER;
};

#endif
