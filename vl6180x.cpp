#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include <ros.h>
#include <james_helper_msgs/RangeButton.h>
#include "delay.h"
#include "debug.h"
#include "raw_button.h"
#include "vl6180x.h"
#include "pins.h"

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

  output_queue_ = output_queue;
  resetState();
  resetDriver();
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  create_task("VL6180X_driver");
}

void Vl6180x::resetDriver()
{
  digitalWrite(PIN_VL6180X_ENABLE, LOW);
  vNopDelayMS(10);  // Can be called before scheduler started!
  digitalWrite(PIN_VL6180X_ENABLE, HIGH);
  vNopDelayMS(10);

  driver_.setDelayFunction(taskDelayMs);
  if (!driver_.begin())
  {
    debug("Proximity sensor VL6180X not found!\n");
  }
  driver_.write8(VL6180X_REG_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);  // default 0x30
  driver_.write8(VL6180X_REG_SYSRANGE_MAX_CONVERGENCE_TIME, 7);  // Units ms. default 49 decimal
}

struct Config
{
  int start_delay_samples = 200;
  int down_thresh = 7;
  int up_thresh = 5;
  long down_latch_dur = 500;
  long down_timeout = 2000;
  int clip_min = 0;
  int clip_max = 100;
  int valid_min = 0;
  int valid_max = 100;

};
Config cfg; // { 200 };

bool Vl6180x::isValidSample(int val)
{
  return cfg.valid_min <= val && val <= cfg.valid_max;
}

int Vl6180x::clipSample(int val)
{
  if (val > cfg.clip_max)
    return cfg.clip_max;
  if (val < cfg.clip_min)
    return cfg.clip_min;
  return val;
}

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
    if ((raw_up && downDuration() > cfg.down_latch_dur)
        || downDuration() >= 2 * cfg.down_timeout)
    {
      is_button_down_ = false;
      begin_down_stamp_ = now();
      begin_down_val_ = 0;
    }
  }
  return is_button_down_;
}

void Vl6180x::updateUI()
{
  digitalWrite(PIN_LED_GREEN, isStable() ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, is_button_down_ ? HIGH : LOW);
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

void Vl6180x::resetState()
{
  samples_ = 0;
  is_button_down_ = false;
  begin_down_stamp_ = 0;
  begin_down_val_ = 0.0;
  memset(diffs_, 0, sizeof(diffs_));
  for (int i = 0; i < EMA_COUNT; i++)
  {
    emas_[i].reset();
  }
  updateUI();
}

void Vl6180x::task()
{
  TickType_t start_stamp = xTaskGetTickCount() - 1;
  int reset_count = 0;

  debug("VL6180X entry");
  while (true)
  {
    digitalWrite(13, HIGH);
    int cur_prox = driver_.readRange();
    digitalWrite(13, LOW);

    if (driver_.driver_status != 0)
    {
      debug("ERROR: PROX READ FAILED. Status: %d", driver_.driver_status);
      tone(PIN_BUZZER, 880, 5000);
      resetDriver();
      resetState();
      ++reset_count;
      taskDelayMs(5000);
      debug("Reset Complete");
      continue;
    }

    if (samples_ % 200 == 0)
      tone(PIN_BUZZER, 220, 3);
    TickType_t cur_stamp = xTaskGetTickCount();
    int rate = samples_ * 1000 / (cur_stamp - start_stamp);
    ++samples_;

    if (!isValidSample(cur_prox))
    {
      if (samples_ % 50 == 0)
        debug("Invalid Reading");
      continue;
    }

    cur_prox = clipSample(cur_prox);
    float cur_proxf = static_cast<float>(cur_prox);

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

    updateUI();

    publishRangeButton(cur_prox);
    if (samples_ % 50 == 0)
    {
      debug("prox: %d rate: %d down: %d stable: %d emas: %d %d %d resets: %d\n", cur_prox, rate,
            is_down, button_msg_.is_stable, emas_[0].emai(), emas_[1].emai(), emas_[1].emai(),
            reset_count);
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

