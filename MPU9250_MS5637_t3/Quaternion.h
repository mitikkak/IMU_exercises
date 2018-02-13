
#pragma once

enum class QuaternionResult : uint8_t
{
    success = 0,
    accelerationFailure,
    magneticMomentFailure
};
QuaternionResult MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
extern float quaternion[4];

