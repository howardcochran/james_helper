#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include <ros.h>
#include <james_helper_msgs/RangeButton.h>
#include "delay.h"
#include "debug.h"
#include "raw_button.h"
#include "vl6180x.h"

#define IN_BETWEEN 2
Vl6180x::Vl6180x()
  : button_pub_("finger_button", &button_msg_)
{
}

void Vl6180x::init(QueueHandle_t output_queue, ros::NodeHandle& nh)
{
  nh_ = &nh;
  nh_->advertise(button_pub_);
  memset(&button_msg_, 0, sizeof(button_msg_));
  button_msg_.emas_length = button_msg_.ema_periods_length = EMA_COUNT;
  button_msg_.emas = emas_;
  button_msg_.ema_periods = ema_periods_;
  button_msg_.misc = misc_;

  output_queue_ = output_queue;
  filtered_state_ = IN_BETWEEN;
  samples_ = 0;
  if (!driver_.begin())
  {
    debug("Proximity sensor VL6180X not found!\n");
  }
  //driver_.setDelayFunction(taskDelayMs);
  driver_.write8(VL6180X_REG_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);  // default 0x30
  driver_.write8(VL6180X_REG_SYSRANGE_MAX_CONVERGENCE_TIME, 7);  // Units ms. default 49 decimal
  driver_.setDelayFunction(taskDelayMs);

  create_task("VL6180X_driver");
}

class Ema
{
protected:
  int period_;
  float alpha_;
  float ema_;

public:
  Ema(int period)
      : period_(period), alpha_(1.0 / period), ema_(NAN)
  {
  }

  void new_data(float val)
  {
    if (isnan(ema_))
      ema_ = val;
    else
      ema_ = alpha_ * val + (1.0 - alpha_) * ema_;
  }

  void reset() { ema_ = NAN; }

  float ema() const { return ema_; }
};

struct Config
{
  int _start_delay_samples;
};
Config cfg { 200 };

void Vl6180x::task()
{
  TickType_t start_stamp = xTaskGetTickCount() - 1;
  TickType_t down_stamp = 0;
  TickType_t raw_change_stamp = 0;
  int prev_raw_state = IN_BETWEEN;
  Ema emas[] = {Ema(20), Ema(50), Ema(200)};

  while (true)
  {
    int cur_prox = driver_.readRange();
    TickType_t cur_stamp = xTaskGetTickCount();
    int rate = samples_ * 1000 / (cur_stamp - start_stamp);
    ++samples_;

    if (samples_ % 20 == 0)
      debug("prox: %d rate: %d\n", cur_prox, rate);
    taskDelayMs(5);
  }
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}

#if 0
void Vl6180x::task()
{
  TickType_t start_stamp = xTaskGetTickCount() - 1;
  int count = 0;
  float ema = 0;
  float ema_alpha = (1.0 / 400.0);
  float ema0 = 0;
  float ema0_alpha = (1.0 / 4.0);
  const int down_thresh = -60;
  const int up_thresh = 50;
  TickType_t down_stamp = 0;
  TickType_t raw_change_stamp = 0;
  int prev_raw_state = IN_BETWEEN;
  int debounce_time = 200;

  while (true)
  {
    int cur_prox = driver_.readRange();
    TickType_t cur_stamp = xTaskGetTickCount();
    int rate = count * 1000 / (cur_stamp - start_stamp);
    ++count;

    ema = ema * (1.0 - ema_alpha) + cur_prox * ema_alpha;
    ema0 = ema0 * (1.0 - ema0_alpha) + cur_prox * ema0_alpha;
    int delta = cur_prox - (int)ema;

    int raw_state = IN_BETWEEN;
    if (delta < down_thresh)
      raw_state = DOWN;
    else if (delta > up_thresh)
      raw_state = UP;

    if (prev_raw_state != raw_state)
    {
      prev_raw_state = raw_state;
      raw_change_stamp = cur_stamp;
    }
    TickType_t raw_change_elapsed = cur_stamp - raw_change_stamp;
    if (raw_change_elapsed > debounce_time)
    {
      filtered_state_ = raw_state;
    }

    range_msg_.header.stamp = nh_->now();
    range_msg_.range = (float)ema0;
    range_pub_.publish(&range_msg_);

    if (count % 20 == 0)
      debug("cur: %d filt: %d prox: %d ema: %d delta: %d rate: %d t: %d %d\n", raw_state, filtered_state_, cur_prox, (int)ema, delta, rate, driver_.time1, driver_.time2);
    taskDelayMs(5);
  }
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}
#endif

int Vl6180x::getState()
{
  if (filtered_state_ == DOWN)
    return DOWN;
  else
    return UP;  // IN_BETWEEN is UP
}

