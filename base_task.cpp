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
  Serial.println(name);
  debug_earlywait("name len %d\n", strlen(name));
  debug_earlywait("name addr 0x%x\n", (void *)name);
  debug_earlywait("create 1: %s\n", name);
  debug_earlywait("entry 0x%x, stack %d, thisptr 0x%x, priority %d, handleptr 0x%x\n",
      (void *)taskEntryWrapper,
      stack_size,
      (void *)this,
      priority,
      (void *)&task_handle_);
  xTaskCreate(
      taskEntryWrapper,
      name,
      stack_size,
      this,
      static_cast<UBaseType_t>(priority),
      &task_handle_);
  debug_earlywait("create end: %s\n", name);
}

void BaseTask::taskEntryWrapper(void *instance)
{
  debug_wait("wrapper\n");
  debug_wait("wrap 0x%x\n", instance);
//  debug_wait("task1 0x%x\n", (void *)&(static_cast<BaseTask*>(instance))->task);
  debug_wait("task2 0x%x\n", (void *)&Morse::task);
  BaseTask* p = (static_cast<BaseTask*>(instance));
  debug_wait("dyncast p 0x%x\n", (void *)p);

  (static_cast<BaseTask*>(instance))->task();
}
