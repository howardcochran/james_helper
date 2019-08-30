#include <FreeRTOS_SAMD21.h> //samd21
#include <queue.h>
#include <Arduino.h>
#include "debug.h"
#include "button.h"
#include "delay.h"
#include "morse.h"

const Morse::MorseMapping Morse::MORSE_MAP[] = {
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
  {"", '\0'}  // sentinel
};

void Morse::init(QueueHandle_t input_queue)
{
  input_queue_ = input_queue;
  unit_ms_ = 100;
  max_dot_duration_ = 2 * unit_ms_;
  max_dash_duration_ = 10 * unit_ms_;
  max_sub_letter_gap_duration_ = 2 * unit_ms_;
  max_letter_gap_duration_ = 5 * unit_ms_;
  create_task("morse");
  debug("morse init\n");
}

int Morse::classifyEvent(ButtonEvent event)
{
  if (event.state == UP)
  {
    if (event.duration <= max_dot_duration_)
    {
      return TOKEN_DOT;
    }
    else if (event.duration <= max_dash_duration_)
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
    if (event.duration <= max_sub_letter_gap_duration_)
    {
      return TOKEN_SUB_LETTER_GAP;
    }
    else if (event.duration <= max_letter_gap_duration_)
    {
      return TOKEN_LETTER_GAP;
    }
    else
    {
      return TOKEN_WORD_GAP;
    }
  }
}

char Morse::decodeTokens(char* tokens)
{
  for (int i = 0; MORSE_MAP[i].tokens[0] != '\0'; i++)
  {
    if (!strcmp(tokens, MORSE_MAP[i].tokens))
    {
      return MORSE_MAP[i].character;
    }
  }
  return '!';
}

void Morse::task()
{
  const int TOKEN_BUF_SIZE = 16;
  char tokens[TOKEN_BUF_SIZE + 1];
  int index = 0;
  ButtonEvent event = {UP, 0};

  while(true)
  {
    EventToken token;
    if (!xQueueReceive(input_queue_, &event, max_letter_gap_duration_))
    {
      token = (event.state == UP) ? TOKEN_WORD_GAP : TOKEN_BACKSPACE;
    }
    else
    {
      token = (EventToken)classifyEvent(event);
    }
    if (index >= TOKEN_BUF_SIZE)
    {
      debug("Token buffer limit exceeded; resetting index to 0\n");
      index = 0;
    }
    // debug("Recv State: %d dur: %d token: %d\n", event.state, event.duration, (int)token);
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
        //debug("%s\n", tokens);
        debug("%c", decodeTokens(tokens));
        index = 0;
        if (token == TOKEN_WORD_GAP)
        {
          debug(" ");
        }
      }
    }
    else if (token == TOKEN_BACKSPACE)
    {
      debug("\nBACKSPACE\n");
    }
  }
}
