#ifndef __JAMES_HELPER_H
#define __JAMES_HELPER_H
#include "morse.h"
#include "nurse_call.h"
#include "hardware_button.h"

#define ERROR_LED_PIN 13 //Led Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE LOW // the state that makes the led light up on your board, either low or high
#define MAIN_BUTTON_PIN 5
#define BUZZER_PIN 14

class App
{
public:
  enum class AppMode
  {
    NURSE_CALL,
    MORSE_KEYBOARD,
    HARDWARE_BUTTON,
    BT_BUTTON,
    MENU,
    MODE_COUNT,
  };

  void init(void);
  void set_major_mode(AppMode);

protected:
  void suspend_all_modes(void);
  AppMode cur_mode_;
  MajorMode* modes_[(int)AppMode::MODE_COUNT];
  Morse morse_decoder_;
  NurseCall nurse_caller_;
  HardwareButton hardware_button_;
};

#endif
