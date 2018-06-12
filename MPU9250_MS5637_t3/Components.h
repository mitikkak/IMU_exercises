
#pragma once
#include "LiquidCrystal_I2C.h"

#ifdef ADAFRUIT_LCD
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);
#else
//extern LiquidCrystal display;
extern LiquidCrystal_I2C display;
#endif

extern float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
extern float gyroBias[3], accelBias[3], magBias[3], magScale[3];
extern float magCalibration[3];
extern uint16_t Pcal[8];
extern volatile bool newData;

#ifdef TEENSY
#include "i2c_t3.h"
#else
#define I2C_NOSTOP 0
#endif
