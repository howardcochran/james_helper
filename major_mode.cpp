#include <FreeRTOS_SAMD21.h> //samd21
#include "major_mode.h"
#include "delay.h"

char out_buf[64];

void MajorMode::suspend(void)
{
  if (task_handle_)
    vTaskSuspend(task_handle_);
}

void MajorMode::resume(void)
{
  if (task_handle_)
    vTaskResume(task_handle_);
}

void MajorMode::create_task(const char *name, int stack_size, int priority)
{
  xTaskCreate(
      taskEntryWrapper,
      name,
      stack_size,
      this,
      static_cast<UBaseType_t>(priority),
      &task_handle_);
}

void MajorMode::taskEntryWrapper(void *instance)
{
  (static_cast<MajorMode*>(instance))->task();
}
