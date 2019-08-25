#ifndef __JAMES_HELPER__MORSE_H
#define __JAMES_HELPER__MORSE_H
enum EventToken
{
  TOKEN_DOT,
  TOKEN_DASH,
  TOKEN_BACKSPACE,
  TOKEN_SUB_LETTER_GAP,
  TOKEN_LETTER_GAP,
  TOKEN_WORD_GAP
};

class Morse
{
public:
  struct MorseMapping
  {
    char *tokens;
    char character;
  };

  Morse(QueueHandle_t input_queue);
  void task(void);
  static void taskEntry(void *instance);
  char decodeTokens(char* tokens);
  int classifyEvent(ButtonEvent event);

protected:
  static const MorseMapping MORSE_MAP[];
  QueueHandle_t input_queue_;
  int unit_ms_;
  int max_dot_duration_;
  int max_dash_duration_;
  int max_sub_letter_gap_duration_;
  int max_letter_gap_duration_;
};

#endif
