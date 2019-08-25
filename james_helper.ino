#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include "james_helper.h"
#include "button.h"
#include "gpio_button.h"
#include "delay.h"
#include "morse.h"

/************** Type Defines and Constants **************/

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
  Morse morse_decoder(trigger_queue);

  xTaskCreate(Button::taskEntry, "buttonRead", 256, &main_trigger, tskIDLE_PRIORITY + 3, &TaskHandle_buttonRead);
  xTaskCreate(Morse::taskEntry, "morse", 256, &morse_decoder, tskIDLE_PRIORITY + 2, &TaskHandle_morse);
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
