#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include "james_helper.h"
#include "button.h"
#include "gpio_button.h"
#include "delay.h"

/************** Type Defines and Constants **************/

const int unit_ms = 100;
const int max_dot_duration = 2 * unit_ms;
const int max_dash_duration = 10 * unit_ms;
const int max_sub_letter_gap_duration = 2 * unit_ms;
const int max_letter_gap_duration = 5 * unit_ms;

//**************************************************************************
// global variables
//**************************************************************************


#if 0
static void testTask(void *pvParameters)
{
  (void)pvParameters;
  char out_buf[64];

  Serial.println("Thread B: Started");
  while(true)
  {
    ButtonEvent event;
    if (!xQueueReceive(button_queue, &event, 30 * 1000))
    {
      Serial.println("No button events detected");
    }
    else
    {
      sprintf(out_buf, "Recv State: %d elapsed: %d\n", event.state, event.duration);
      Serial.print(out_buf);
    }
  }
}
#endif

struct MorseMapping
{
  char *tokens;
  char character;
};

const MorseMapping MORSE_MAP[] = {
  {".-", 'A'},
  {"-...", 'B'},
  {"-.-.", 'C'},
  {"-..", 'D'},
  {".", 'E'},
  {"..-.", 'F'},
  {"--.", 'G'},
  {"....", 'H'},
  {"..", 'I'},
  {".---", 'J'},
  {"-.-", 'K'},
  {".-..", 'L'},
  {"--", 'M'},
  {"-.", 'N'},
  {"---", 'O'},
  {".--.", 'P'},
  {"--.-", 'Q'},
  {".-.", 'R'},
  {"...", 'S'},
  {"-", 'T'},
  {"..-", 'U'},
  {"...-", 'V'},
  {".--", 'W'},
  {"-..-", 'X'},
  {"-.--", 'Y'},
  {"--..", 'Z'},
  {".-.-.-", '.'},
  {".----", '1'},
  {"..---", '2'},
  {"...--", '3'},
  {"....-", '4'},
  {".....", '5'},
  {"-....", '6'},
  {"--...", '7'},
  {"---..", '8'},
  {"----.", '9'},
  {"-----", '0'},
};

enum EventToken
{
  TOKEN_DOT,
  TOKEN_DASH,
  TOKEN_BACKSPACE,
  TOKEN_SUB_LETTER_GAP,
  TOKEN_LETTER_GAP,
  TOKEN_WORD_GAP
};

static int classifyEvent(ButtonEvent event)
{
  if (event.state == UP)
  {
    if (event.duration <= max_dot_duration)
    {
      return TOKEN_DOT;
    }
    else if (event.duration <= max_dash_duration)
    {
      return TOKEN_DASH;
    }
    else
    {
      return TOKEN_BACKSPACE;
    }
  }
  else
  {
    if (event.duration <= max_sub_letter_gap_duration)
    {
      return TOKEN_SUB_LETTER_GAP;
    }
    else if (event.duration <= max_letter_gap_duration)
    {
      return TOKEN_LETTER_GAP;
    }
    else
    {
      return TOKEN_WORD_GAP;
    }
  }
}

char decodeTokens(char* tokens) {
  for (int i = 0; i < (sizeof(MORSE_MAP) / sizeof(MorseMapping)); i++)
  {
    if (!strcmp(tokens, MORSE_MAP[i].tokens))
    {
      return MORSE_MAP[i].character;
    }
  }
  return '!';
}

static void morseTaskEntry(void *params)
{
  QueueHandle_t input_queue = (QueueHandle_t *)params;
  char out_buf[64];
  const int TOKEN_BUF_SIZE = 16;
  char tokens[TOKEN_BUF_SIZE + 1];
  int index = 0;
  ButtonEvent event = {UP, 0};

  while(true)
  {
    EventToken token;
    if (!xQueueReceive(input_queue, &event, max_letter_gap_duration))
    {
      token = (event.state == UP) ? TOKEN_WORD_GAP : TOKEN_BACKSPACE;
    }
    else
    {
      token = (EventToken)classifyEvent(event);
    }
    if (index >= TOKEN_BUF_SIZE)
    {
      Serial.println("Token buffer limit exceeded; resetting index to 0");
      index = 0;
    }
    //sprintf(out_buf, "Recv State: %d dur: %d token: %d\n", event.state, event.duration, (int)token);
    //Serial.print(out_buf);
    if (token == TOKEN_DOT)
    {
      tokens[index++] = '.';
    }
    else if (token == TOKEN_DASH)
    {
      tokens[index++] = '-';
    }
    else if (token == TOKEN_LETTER_GAP || token == TOKEN_WORD_GAP)
    {
      if (index > 0)
      {
        tokens[index] = '\0';
        //Serial.println(tokens);
        Serial.print(decodeTokens(tokens));
        index = 0;
        if (token == TOKEN_WORD_GAP)
        {
          Serial.print(' ');
        }
      }
    }
    else if (token == TOKEN_BACKSPACE)
    {
      Serial.println("\nBACKSPACE");
    }
  }
}

//*****************************************************************

void setup()
{

  Serial.begin(115200);

  vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
  while (!Serial) ;  // Wait for serial terminal to open port before starting program

  Serial.println("******************************");

  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object.
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack!
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  QueueHandle_t trigger_queue = xQueueCreate(20, sizeof(ButtonEvent));
  GPIOButton raw_trigger(MAIN_BUTTON_PIN);
  Button main_trigger(raw_trigger, trigger_queue);
  xTaskCreate(Button::taskEntry, "buttonRead", 256, &main_trigger, tskIDLE_PRIORITY + 3, &TaskHandle_buttonRead);
  xTaskCreate(morseTaskEntry, "morse", 256, trigger_queue, tskIDLE_PRIORITY + 2, &TaskHandle_morse);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
	vTaskStartScheduler();
}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop()
{
    vNopDelayMS(1000);
}


//*****************************************************************
