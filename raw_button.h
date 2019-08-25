#ifndef __JAMES_HELPER__RAW_BUTTON_H
#define __JAMES_HELPER__RAW_BUTTON_H

#define DOWN LOW
#define UP HIGH

class RawButton
{
public:
  virtual int getState(void) = 0;
  virtual void init(void) {}
  virtual void deInit(void) {}
};
#endif

