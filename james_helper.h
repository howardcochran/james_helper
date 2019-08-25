#ifndef __JAMES_HELPER_H
#define __JAMES_HELPER_H

#define ERROR_LED_PIN 13 //Led Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE LOW // the state that makes the led light up on your board, either low or high
#define MAIN_BUTTON_PIN 5
#define BUZZER_PIN 14

TaskHandle_t TaskHandle_buttonRead;
TaskHandle_t TaskHandle_morse;
// TaskHandle_t TaskHandle_monitor;
#endif
