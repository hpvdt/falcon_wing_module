# Spar Sensor Firmware

Firmware meant to run on the spar strain sensor boards. The purpose of this code is simple: read the HX711 and provide the most recent value over the one wire interface when requested of the board.

***Although the addresses of the boards were originally meant to be set by means of resistive dividers `R1` and `R3`, they are currently hardcoded as part of uploading.***

### Note on multiple targets

Although the primary target of this project is the spar sensor boards themselves, there are provisions in this code to have it compiled and run on an Arduino Nano for testing. Hence why there are so many `#ifdef...#else...#endif` structure through the code to reroute the logic for the target hardware automatically.

