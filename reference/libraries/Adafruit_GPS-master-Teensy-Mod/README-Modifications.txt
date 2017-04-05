This revision of the library has been adapted in order to support the Teensy 3.X series microcontrollers, removing NewSoftSerial references that cause incompatibilities when compiling the program.

Please note to REMOVE the original Afafruit GPS library when using this modification, otherwise you might end up with the same error.

Spelling to be corrected at some point.

This is based off the efforts of rvnash and kingforger

https://forum.pjrc.com/threads/95-GPS-working!
https://forum.pjrc.com/threads/24979-Teensy-3-1-Ultimate-GPS-code

Please remember to also update the WString.cpp file and other files found in the hardware folder, instructions in that README.txt file.

Also note that you "may" have to relocate the SD card folder to ensure the teensy version is used, however this could just be an issue with my IDE rather than a common one.