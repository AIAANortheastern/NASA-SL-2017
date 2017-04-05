Replace file found in

/arduino-1.0.1/hardware/teensy/avr/cores/teensy3

avr_functions.h removes static asignments of itoa to allow for modification to run

Wstring has a modified itoa function that is compatible with the Teensy 3.6