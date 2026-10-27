// Types used in function signatures must live in a header: the Arduino IDE inserts auto-generated
// prototypes ABOVE any struct defined in the .ino file itself.
#pragma once
#include <Arduino.h>
struct Row { int y; uint8_t size; uint16_t *col; char text[64]; int off; uint32_t next; };
