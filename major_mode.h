#ifndef __JAMES_HELPER__MAJOR_MODE_H
#define __JAMES_HELPER__MAJOR_MODE_H

class MajorMode
{
public:
  virtual void create_task(const char *name, int stack_size=256, int priority=2);
  virtual void task(void) = 0;
  virtual void suspend(void);
  virtual void resume(void);
  virtual TaskHandle_t getTaskHandle() { return task_handle_; }
  static void taskEntryWrapper(void *instance);
protected:
  TaskHandle_t task_handle_;
};
#endif
