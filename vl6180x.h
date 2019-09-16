#ifndef __JAMES_HELPER__VL6180X_H
#define __JAMES_HELPER__VL6180X_H
#include <ros.h>
#include <sensor_msgs/Range.h>
#include "Adafruit_VL6180X.h"
#include "base_task.h"
#include "raw_button.h"

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
  sensor_msgs::Range range_msg_;
  ros::Publisher range_pub_;
  ros::NodeHandle* nh_;
};

#endif
