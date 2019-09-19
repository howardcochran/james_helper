#ifndef __JAMES_HELPER__VL6180X_H
#define __JAMES_HELPER__VL6180X_H
#include <ros.h>
#include <james_helper_msgs/RangeButton.h>
#include "Adafruit_VL6180X.h"
#include "base_task.h"
#include "raw_button.h"

#define EMA_COUNT 3

class Vl6180x : public BaseTask, public RawButton
{
public:
  Vl6180x();
  void init(QueueHandle_t input_queue, ros::NodeHandle& nh);
  void task(void);
  int getState();

protected:
  QueueHandle_t output_queue_;
  Adafruit_VL6180X driver_;
  int filtered_state_;
  ros::NodeHandle* nh_;
  ros::Publisher button_pub_;
  james_helper_msgs::RangeButton button_msg_;
  float emas_[EMA_COUNT];
  float ema_periods_[EMA_COUNT];
  float misc_[0];
  int samples_;
};

#endif
