#ifndef __JAMES_HELPER__GPIO_BUTTON_H
#define __JAMES_HELPER__GPIO_BUTTON_H
#include "raw_button.h"

class GPIOButton : public RawButton
{
public:
  GPIOButton(int pin);
  virtual int getState();

protected:
  int pin_;
};
#endif
