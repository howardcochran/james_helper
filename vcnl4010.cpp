#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include "delay.h"
#include "debug.h"
#include "raw_button.h"
#include "vcnl4010.h"

#define IN_BETWEEN 2
Vcnl4010::Vcnl4010()
  : range_pub_("range_data", &range_msg_)
{
}

void Vcnl4010::init(QueueHandle_t output_queue, ros::NodeHandle& nh)
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
  if (!vcnl_.begin(nh))
  {
    debug("Proximity sensor VNCL4010 not found!\n");
  }
  vcnl_.setDelayFunction(taskDelayMs);
  create_task("VNCL4010_driver");
}

void Vcnl4010::task()
{
  TickType_t start_stamp = xTaskGetTickCount() - 1;
  int count = 0;
  float ema = 300;
  float ema_alpha = (1.0 / 400.0);
  const int down_thresh = -60;
  const int up_thresh = 50;
  TickType_t down_stamp = 0;
  TickType_t raw_change_stamp = 0;
  int prev_raw_state = IN_BETWEEN;
  int debounce_time = 200;

  while (true)
  {
    int raw_prox = vcnl_.readProximity();
    int cur_prox = (int)(10 * 65536 / raw_prox);
    TickType_t cur_stamp = xTaskGetTickCount();
    int rate = count * 1000 / (cur_stamp - start_stamp);
    ++count;

    ema = ema * (1.0 - ema_alpha) + cur_prox * ema_alpha;
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
    range_msg_.range = (float)cur_prox;
    range_pub_.publish(&range_msg_);

    if (count % 100 == 0)
      debug("cur: %d filt: %d prox: %d ema: %d delta: %d rate: %d\n", raw_state, filtered_state_, cur_prox, (int)ema, delta, rate);
    taskDelayMs(5);
  }
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}

int Vcnl4010::getState()
{
  if (filtered_state_ == DOWN)
    return DOWN;
  else
    return UP;  // IN_BETWEEN is UP
}

