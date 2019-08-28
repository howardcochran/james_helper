#ifndef __JAMES_HELPER__VNCL4010_H
#define __JAMES_HELPER__VNCL4010_H
#include "Adafruit_VCNL4010.h"
#include "base_task.h"

class Vcnl4010 : public BaseTask
{
public:
  void init(QueueHandle_t input_queue);
  void task(void);

protected:
  QueueHandle_t output_queue_;
  Adafruit_VCNL4010 vcnl_;
};

#endif
