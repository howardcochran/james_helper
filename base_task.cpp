#include <FreeRTOS_SAMD21.h> //samd21
#include "base_task.h"
#include "delay.h"

char out_buf[64];

void BaseTask::suspend(void)
{
  if (task_handle_)
    vTaskSuspend(task_handle_);
}

void BaseTask::resume(void)
{
  if (task_handle_)
    vTaskResume(task_handle_);
}

void BaseTask::create_task(const char *name, int stack_size, int priority)
{
  xTaskCreate(
      taskEntryWrapper,
      name,
      stack_size,
      this,
      static_cast<UBaseType_t>(priority),
      &task_handle_);
}

void BaseTask::taskEntryWrapper(void *instance)
{
  (static_cast<MajorMode*>(instance))->task();
}
