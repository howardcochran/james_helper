#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define SERIAL Serial
#define ERROR_LED_PIN 13 //Led Pin: Typical Arduino Board
#define ERROR_LED_LIGHTUP_STATE LOW // the state that makes the led light up on your board, either low or high
#define MAIN_BUTTON_PIN 5
#define BUZZER_PIN 14
#define DOWN LOW
#define UP HIGH

const int unit_ms = 100;
const int max_dot_duration = 2 * unit_ms;
const int max_dash_duration = 10 * unit_ms;
const int max_sub_letter_gap_duration = 2 * unit_ms;
const int max_letter_gap_duration = 5 * unit_ms;

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_buttonReadTask;
TaskHandle_t Handle_testTask;
TaskHandle_t Handle_monitorTask;

class ButtonEvent
{
public:
  int state;
  int duration;
};

QueueHandle_t button_queue;

void taskDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void taskDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void taskDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}

static void buttonReadTask(void *pvParameters)
{
  (void)pvParameters;
  char out_buf[64];
  int prevButtonState = UP;
  TickType_t prevStateChangeTime;
  
  SERIAL.println("buttonReadTask: Started");

  while (true)
  {
    TickType_t curTime = xTaskGetTickCount();
    int curButtonState = digitalRead(MAIN_BUTTON_PIN);
    if (curButtonState != prevButtonState)
    {
      ButtonEvent event;
      TickType_t elapsed = curTime - prevStateChangeTime;
      event.state = curButtonState;
      event.duration = elapsed;
      xQueueSend(button_queue, &event, 0);

      if (curButtonState == DOWN)
      {
        tone(BUZZER_PIN, 110);
      }
      else
      {
        noTone(BUZZER_PIN);
      }
      prevStateChangeTime = curTime;
      prevButtonState = curButtonState;
    }
    taskDelayMsUntil(&curTime, 2);
  }
  
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("Thread A: Deleting");
  vTaskDelete( NULL );
}

static void testTask(void *pvParameters)
{
  (void)pvParameters;
  char out_buf[64];
  
  SERIAL.println("Thread B: Started");
  while(true)
  {
    ButtonEvent event;
    if (!xQueueReceive(button_queue, &event, 30 * 1000))
    {
      SERIAL.println("No button events detected");
    }
    else
    {
      sprintf(out_buf, "Recv State: %d elapsed: %d\n", event.state, event.duration);
      SERIAL.print(out_buf);
    }
  }
}

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

static void morseTask(void *pvParameters)
{
  (void)pvParameters;
  char out_buf[64];
  const int TOKEN_BUF_SIZE = 16;
  char tokens[TOKEN_BUF_SIZE + 1];
  int index = 0;
  ButtonEvent event = {UP, 0};
  
  while(true)
  {
    EventToken token;
    if (!xQueueReceive(button_queue, &event, max_letter_gap_duration))
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
    //SERIAL.print(out_buf);
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

void taskMonitor(void *pvParameters)
{
    int x;
    int measurement;
    
    SERIAL.println("Task Monitor: Started");

    // run this task afew times before exiting forever
    for(x=0; x<10; ++x)
    {

      SERIAL.println("");
      SERIAL.println("******************************");
      SERIAL.println("[Stacks Free Bytes Remaining] ");

      measurement = uxTaskGetStackHighWaterMark( Handle_buttonReadTask );
      SERIAL.print("Thread A: ");
      SERIAL.println(measurement);
      
      measurement = uxTaskGetStackHighWaterMark( Handle_testTask );
      SERIAL.print("Thread B: ");
      SERIAL.println(measurement);
      
      measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
      SERIAL.print("Monitor Stack: ");
      SERIAL.println(measurement);

      SERIAL.println("******************************");

      taskDelayMs(10000); // print every 10 seconds
    }

    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    SERIAL.println("Task Monitor: Deleting");
    vTaskDelete( NULL );

}

//*****************************************************************

void setup() 
{

  SERIAL.begin(115200);

  vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
  while (!SERIAL) ;  // Wait for serial terminal to open port before starting program

  SERIAL.println("******************************");

  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. 
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! 
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);
  pinMode(MAIN_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  
  button_queue = xQueueCreate(20, sizeof(ButtonEvent));
  xTaskCreate(buttonReadTask, "buttonRead", 256, NULL, tskIDLE_PRIORITY + 3, &Handle_buttonReadTask);
  xTaskCreate(morseTask, "test", 256, NULL, tskIDLE_PRIORITY + 2, &Handle_testTask);
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
