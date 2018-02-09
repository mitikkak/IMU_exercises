
#include <gtest/gtest.h>
#include "Loop.h"

class TestMadgwickQuaternionUpdate : public ::testing::Test {
public:

    void SetUp() {
    }

    void TearDown() {

    }
};

struct Directions
{
    Directions(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_)
    {}
    float x{0.0};
    float y{0.0};
    float z{0.0};
};

TEST_F(TestMadgwickQuaternionUpdate, accelerationFailure)
{
    Directions acceleration(0.0, 0.0, 0.0);
    Directions rotationRate(0.0, 0.0, 0.0);
    Directions magneticMoment(0.0, 0.0, 0.0);
    QuaternionResult const retValue = MadgwickQuaternionUpdate(acceleration.x, acceleration.y, acceleration.z,
                             rotationRate.x, rotationRate.y, rotationRate.z,
                             magneticMoment.x, magneticMoment.y, magneticMoment.z);
    ASSERT_EQ(QuaternionResult::accelerationFailure, retValue);
    EXPECT_FLOAT_EQ(1.0f, quaternion[0]);
    EXPECT_FLOAT_EQ(0.0f, quaternion[1]);
    EXPECT_FLOAT_EQ(0.0f, quaternion[2]);
    EXPECT_FLOAT_EQ(0.0f, quaternion[3]);
}
TEST_F(TestMadgwickQuaternionUpdate, magneticMomentFailure)
{
    Directions acceleration(1.0, -5.0, -10.0);
    Directions rotationRate(0.0, 0.0, 0.0);
    Directions magneticMoment(0.0, 0.0, 0.0);
    QuaternionResult const retValue = MadgwickQuaternionUpdate(acceleration.x, acceleration.y, acceleration.z,
                             rotationRate.x, rotationRate.y, rotationRate.z,
                             magneticMoment.x, magneticMoment.y, magneticMoment.z);
    ASSERT_EQ(QuaternionResult::magneticMomentFailure, retValue);
    EXPECT_FLOAT_EQ(1.0f, quaternion[0]);
    EXPECT_FLOAT_EQ(0.0f, quaternion[1]);
    EXPECT_FLOAT_EQ(0.0f, quaternion[2]);
    EXPECT_FLOAT_EQ(0.0f, quaternion[3]);
}
TEST_F(TestMadgwickQuaternionUpdate, success)
{
    Directions acceleration(10.5, 20.5, 30.5);
    Directions rotationRate(190.6, 10.6, 10.6);
    Directions magneticMoment(20.7, 20.7, 20.7);
    microsecs const Now = 1000;
    microsecs const lastUpdate = 0;
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    QuaternionResult const retValue = MadgwickQuaternionUpdate(acceleration.x, acceleration.y, acceleration.z,
                             rotationRate.x, rotationRate.y, rotationRate.z,
                             magneticMoment.x, magneticMoment.y, magneticMoment.z);
    ASSERT_EQ(QuaternionResult::success, retValue);
    EXPECT_FLOAT_EQ(0.9954569, quaternion[0]);
    EXPECT_FLOAT_EQ(0.09492203, quaternion[1]);
    EXPECT_FLOAT_EQ(0.005259834, quaternion[2]);
    EXPECT_FLOAT_EQ(0.005257486, quaternion[3]);
}
