#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include <ros.h>
#include <james_helper_msgs/RangeButton.h>
#include "delay.h"
#include "debug.h"
#include "raw_button.h"
#include "vl6180x.h"

#define BUZZER_PIN 14

static long now()
{
  return static_cast<long>(xTaskGetTickCount());
}

Vl6180x::Vl6180x()
  : button_pub_("finger_button", &button_msg_),
    emas_{Ema(20), Ema(50), Ema(200)}
{
}

void Vl6180x::init(QueueHandle_t output_queue, ros::NodeHandle& nh)
{
  nh_ = &nh;
  memset(msg_emas_, 0, sizeof(msg_emas_));
  memset(msg_ema_periods_, 0, sizeof(msg_ema_periods_));
  memset(diffs_, 0, sizeof(diffs_));
  button_msg_.emas_length = button_msg_.ema_periods_length = EMA_COUNT;
  button_msg_.emas = msg_emas_;
  button_msg_.ema_periods = msg_ema_periods_;
  button_msg_.misc_length = sizeof(diffs_) / sizeof(diffs_[0]);
  button_msg_.misc = diffs_;
  nh_->advertise(button_pub_);

  samples_ = 0;
  is_button_down_ = false;
  begin_down_stamp_ = 0;
  begin_down_val_ = 0.0;

  output_queue_ = output_queue;
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

struct Config
{
  int start_delay_samples = 200;
  int down_thresh = 5;
  int up_thresh = 3;
  long down_latch_dur = 500;
  long down_timeout = 2000;

};
Config cfg; // { 200 };

void Vl6180x::updateEmas(float val)
{
  for (int i = 0; i < EMA_COUNT; i++)
  {
    emas_[i].new_data(val);
  }
}

void Vl6180x::updateDiffs(float val)
{
  diffs_[0] = val - emas_[0].ema();
  for (int i = 1; i < EMA_COUNT; i++)
  {
    diffs_[i] = emas_[i-1].ema() - emas_[i].ema();
  }
}

long Vl6180x::downDuration()
{
  if (begin_down_stamp_)
    return now() - begin_down_stamp_;
  else
    return 0;
}

bool Vl6180x::isDownTimeout()
{
  return downDuration() > cfg.down_timeout;
}

bool Vl6180x::isStable()
{
  return -8 < diffs_[2] && diffs_[2] < 8
      && -4 < diffs_[2] && diffs_[2] < 4;
}

bool Vl6180x::isTrendDown()
{
  bool is_trend_down = true;
  for (int i = 0; i < EMA_COUNT; i++)
  {
    if (diffs_[i] > 0)
      is_trend_down = false;
  }
  return is_trend_down;
}

bool Vl6180x::updateButtonState(float val)
{
  //float raw_down = (val < emas_[0].ema() - cfg.down_thresh);
  bool raw_down = diffs_[0] < -cfg.down_thresh;
  diffs_[EMA_COUNT] = raw_down ? 80 : 75;
  diffs_[EMA_COUNT+1] = isTrendDown() ? 70 : 65;
  if (!is_button_down_ && raw_down && isTrendDown() && isStable())
  {
    is_button_down_ = true;
    begin_down_stamp_ = now();
    begin_down_val_ = emas_[0].ema();
  }
  else if (is_button_down_)
  {
    bool raw_up = (val > begin_down_val_ - cfg.up_thresh);
    if (raw_up and downDuration() > cfg.down_latch_dur)
    {
      is_button_down_ = false;
      begin_down_stamp_ = now();
      begin_down_val_ = 0;
    }
  }
  return is_button_down_;
}

void Vl6180x::publishRangeButton(float range)
{
  button_msg_.is_down = is_button_down_;
  button_msg_.is_stable = isStable();
  button_msg_.range = range;
  for (int i = 0; i < EMA_COUNT; i++)
  {
    button_msg_.emas[i] = emas_[i].ema();
    button_msg_.ema_periods[i] = emas_[i].period();
  }
  button_pub_.publish(&button_msg_);
}

void Vl6180x::task()
{
  TickType_t start_stamp = xTaskGetTickCount() - 1;
  TickType_t down_stamp = 0;
  TickType_t raw_change_stamp = 0;

  debug("VL6180X entry");
  while (true)
  {
    int cur_prox = driver_.readRange();
    if (samples_ % 200 == 0)
      tone(BUZZER_PIN, 220, 3);
    float cur_proxf = static_cast<float>(cur_prox);
    TickType_t cur_stamp = xTaskGetTickCount();
    int rate = samples_ * 1000 / (cur_stamp - start_stamp);
    ++samples_;

    if (samples_ < cfg.start_delay_samples)
    {
      updateEmas(cur_proxf);
      continue;
    }

    bool is_down = updateButtonState(cur_prox);
    if (!is_down || isDownTimeout())
    {
      updateEmas(cur_proxf);
      updateDiffs(cur_proxf);
    }

    publishRangeButton(cur_prox);
    if (samples_ % 50 == 0)
    {
      debug("prox: %d rate: %d down: %d stable: %d emas: %d %d %d\n", cur_prox, rate,
            is_down, button_msg_.is_stable, emas_[0].emai(), emas_[1].emai(), emas_[1].emai());
    }
    taskDelayMs(5);
  }
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}

int Vl6180x::getState()
{
  if (is_button_down_)
    return DOWN;
  else
    return UP;  // IN_BETWEEN is UP
}

