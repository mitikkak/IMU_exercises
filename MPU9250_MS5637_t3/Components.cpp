
#include "Components.h"
LiquidCrystal_I2C display(0x27,16,4);
float aRes, gRes, mRes;
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
volatile bool newData = false;
