#include <gtest/gtest.h>
#include <cmath>
#include "rabcl/controller/body_attitude.hpp"

namespace rabcl
{

TEST(BodyAttitude, ImuMinusJointGivesBody)
{
  BodyAttitude body;
  body.UpdateFromImu({1.0f, 0.2f, -0.1f}, {0.3f, 0.05f, 0.02f});
  const Attitude a = body.Get();
  EXPECT_FLOAT_EQ(a.yaw, 0.7f);
  EXPECT_FLOAT_EQ(a.pitch, 0.15f);
  EXPECT_NEAR(a.roll, -0.12f, 1e-6f);
}

TEST(BodyAttitude, ChassisRateIntegratesYawOnly)
{
  BodyAttitude body;
  body.UpdateFromImu({0.0f, 0.2f, 0.1f}, {0.0f, 0.0f, 0.0f});
  body.UpdateFromChassisRate(2.0f, 0.5f);
  const Attitude a = body.Get();
  EXPECT_FLOAT_EQ(a.yaw, 1.0f);
  EXPECT_FLOAT_EQ(a.pitch, 0.2f);
  EXPECT_FLOAT_EQ(a.roll, 0.1f);
}

TEST(BodyAttitude, YawWrapsWhenIntegrating)
{
  BodyAttitude body;
  body.UpdateFromChassisRate(1.0f, 4.0f);  // 4 rad → 4 − 2π
  EXPECT_NEAR(body.Get().yaw, 4.0f - 2.0f * static_cast<float>(M_PI), 1e-5f);
}

TEST(BodyAttitude, ResetZeroes)
{
  BodyAttitude body;
  body.UpdateFromImu({1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f});
  body.Reset();
  EXPECT_FLOAT_EQ(body.Get().yaw, 0.0f);
  EXPECT_FLOAT_EQ(body.Get().pitch, 0.0f);
  EXPECT_FLOAT_EQ(body.Get().roll, 0.0f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
