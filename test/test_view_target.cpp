#include <gtest/gtest.h>
#include <cmath>
#include "rabcl/controller/view_target.hpp"

namespace rabcl
{

TEST(ViewTarget, AddDeltaAccumulatesAndWrapsYaw)
{
  ViewTarget view;
  view.AddDelta(0.5f, 0.0f);
  view.AddDelta(0.5f, 0.0f);
  EXPECT_FLOAT_EQ(view.Get().yaw, 1.0f);
  view.AddDelta(3.0f, 0.0f);  // 4.0 → 4 − 2π
  EXPECT_NEAR(view.Get().yaw, 4.0f - 2.0f * static_cast<float>(M_PI), 1e-5f);
}

TEST(ViewTarget, PitchClampedToLimits)
{
  ViewTarget view;
  const float pi = static_cast<float>(M_PI);
  view.SetPitchLimits(-pi / 12.0f, pi / 6.0f);
  view.AddDelta(0.0f, -10.0f);
  EXPECT_FLOAT_EQ(view.Get().pitch, -pi / 12.0f);
  view.AddDelta(0.0f, 20.0f);
  EXPECT_FLOAT_EQ(view.Get().pitch, pi / 6.0f);
  EXPECT_FLOAT_EQ(view.Get().roll, 0.0f);
}

TEST(ViewTarget, ResetSetsTarget)
{
  ViewTarget view;
  view.AddDelta(1.0f, 0.1f);
  view.Reset({0.0f, 0.0f, 0.0f});
  EXPECT_FLOAT_EQ(view.Get().yaw, 0.0f);
  EXPECT_FLOAT_EQ(view.Get().pitch, 0.0f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
