
#pragma once
#include "LiquidCrystal.h"

#ifdef ADAFRUIT_LCD
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);
#else
extern LiquidCrystal display;
#endif
