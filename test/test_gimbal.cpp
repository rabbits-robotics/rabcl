#include <gtest/gtest.h>
#include <cmath>
#include "rabcl/controller/gimbal.hpp"

namespace rabcl
{

TEST(Gimbal, SubtractsBody)
{
  Gimbal gimbal;
  const NeckAngles j = gimbal.Solve({0.5f, 0.1f, -0.2f}, {0.2f, 0.05f, 0.1f});
  EXPECT_FLOAT_EQ(j.yaw, 0.3f);
  EXPECT_FLOAT_EQ(j.pitch, 0.05f);
  EXPECT_NEAR(j.roll, -0.3f, 1e-6f);
}

TEST(Gimbal, YawWrapsAcrossPi)
{
  Gimbal gimbal;
  const NeckAngles j = gimbal.Solve({3.0f, 0.0f, 0.0f}, {-3.0f, 0.0f, 0.0f});
  EXPECT_NEAR(j.yaw, 6.0f - 2.0f * static_cast<float>(M_PI), 1e-5f);  // ≈ −0.283、最短側
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
