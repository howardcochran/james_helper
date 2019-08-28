#include <FreeRTOS_SAMD21.h> //samd21
#include "debug.h"
#include "base_task.h"
#include "delay.h"
#include "morse.h"

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
  (static_cast<BaseTask*>(instance))->task();
}
