
#pragma once

uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
void readMagData(int16_t * destination);
extern float quaternion[4];
extern float deltat;

enum class QuaternionResult : uint8_t
{
    success = 0,
    accelerationFailure,
    magneticMomentFailure
};
QuaternionResult MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

typedef uint32_t microsecs;
