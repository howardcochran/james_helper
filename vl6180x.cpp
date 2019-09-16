#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include "delay.h"
#include "debug.h"
#include "raw_button.h"
#include "vl6180x.h"

#define IN_BETWEEN 2
Vl6180x::Vl6180x()
  : range_pub_("range_data", &range_msg_)
{
}

void Vl6180x::init(QueueHandle_t output_queue, ros::NodeHandle& nh)
{
  nh_ = &nh;
  nh_->advertise(range_pub_);
  range_msg_.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg_.header.frame_id = "prox_sensor";
  range_msg_.field_of_view = 20 * 3.14159 / 180;
  range_msg_.min_range = 0.001;
  range_msg_.max_range = 0.050;

  output_queue_ = output_queue;
  filtered_state_ = IN_BETWEEN;
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

int Vl6180x::getState()
{
  if (filtered_state_ == DOWN)
    return DOWN;
  else
    return UP;  // IN_BETWEEN is UP
}

