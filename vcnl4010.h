#ifndef __JAMES_HELPER__VNCL4010_H
#define __JAMES_HELPER__VNCL4010_H
#include <ros.h>
#include <sensor_msgs/Range.h>
#include "Adafruit_VCNL4010.h"
#include "base_task.h"
#include "raw_button.h"

class Vcnl4010 : public BaseTask, public RawButton
{
public:
  Vcnl4010();
  void init(QueueHandle_t input_queue, ros::NodeHandle& nh);
  void task(void);
  int getState();

protected:
  QueueHandle_t output_queue_;
  Adafruit_VCNL4010 vcnl_;
  int filtered_state_;
  sensor_msgs::Range range_msg_;
  ros::Publisher range_pub_;
  ros::NodeHandle* nh_;
};

#endif
