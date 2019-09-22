#ifndef __JAMES_HELPER__VL6180X_H
#define __JAMES_HELPER__VL6180X_H
#include <ros.h>
#include <james_helper_msgs/RangeButton.h>
#include "Adafruit_VL6180X.h"
#include "base_task.h"
#include "raw_button.h"
#include "debug.h"

#define EMA_COUNT 3

class Ema
{
protected:
  int period_;
  float alpha_;
  float ema_;
  int count_;

public:
  Ema(int period)
      : period_(period), alpha_(1.0 / period), ema_(NAN), count_(0)
  {
  }

  void new_data(float val)
  {
    if (isnan(ema_)) // || ema_ < 0.0)
      ema_ = val;
    else
      ema_ = alpha_ * val + (1.0 - alpha_) * ema_;
  }

  void reset() { ema_ = NAN; }

  float ema() const { return ema_; }
  int emai() const { return static_cast<int>(ema_); }
  int period() const { return period_; }
};

class Vl6180x : public BaseTask, public RawButton
{
public:
  Vl6180x();
  void init(QueueHandle_t input_queue, ros::NodeHandle& nh);
  void task(void);
  int getState();

protected:
  void resetDriver();
  void resetState();
  void updateEmas(float val);
  void updateDiffs(float val);
  bool updateButtonState(float val);
  bool isValidSample(int val);
  int clipSample(int val);
  bool isStable();
  bool isTrendDown();
  long downDuration();
  bool isDownTimeout();
  void publishRangeButton(float range);

  QueueHandle_t output_queue_;
  Adafruit_VL6180X driver_;
  int filtered_state_;
  ros::NodeHandle* nh_;
  ros::Publisher button_pub_;
  james_helper_msgs::RangeButton button_msg_;
  float msg_emas_[EMA_COUNT];
  float msg_ema_periods_[EMA_COUNT];
  float diffs_[EMA_COUNT+2];
  Ema emas_[EMA_COUNT];
  int samples_;
  bool is_button_down_;
  long begin_down_stamp_;
  float begin_down_val_;

};

#endif
