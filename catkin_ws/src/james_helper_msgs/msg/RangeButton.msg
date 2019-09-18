# Data from a virtual button driven by a range (distance) sensor
# Contains the button state (is_down) as well as a bunch of internal filter states
# for tuning and diagnostics

bool is_down           # Is the virtual button in a pressed state
bool is_stable         # Whether the filter has stabilized. No button presses are allowed until true.
float32 range          # Current raw range reading
float32[] emas         # Expoential moving averages of the range, in ascending order of period
float32[] ema_periods  # The periods of the corresponding EMA's in emas array
float32[] misc         # Any misc data that may be useful for debug / graphing
