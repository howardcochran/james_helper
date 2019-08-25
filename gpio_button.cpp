#include <Arduino.h>
#include "gpio_button.h"

GPIOButton::GPIOButton(int pin)
  : pin_(pin)
{
  pinMode(pin, INPUT_PULLUP);
}

int GPIOButton::getState()
{
  return digitalRead(pin_);
}
