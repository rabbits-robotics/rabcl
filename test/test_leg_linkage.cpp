#include <gtest/gtest.h>

#include <cmath>

#include "rabcl/controller/leg_linkage.hpp"

namespace rabcl
{

// gamma の寸法（設計書 2026-10-25-gamma-leg-linkage-design.md 2 章）。rabcl 本体には書かない
constexpr LegLinkageConfig GAMMA = {0.041f, 1.0471976f, 0.03975f, 0.060f, -0.026f, -0.049f,
  0.0f, 0.0175f, 0.044f, -0.0325f, 1.5707963f};

TEST(LegLinkage, ForwardMatchesCadAtZero)
{
  LegLinkage link(GAMMA);
  LegPose p;
  ASSERT_TRUE(link.Forward(0.0f, p));
  EXPECT_NEAR(p.bar_angle, 0.4821f, 1e-3f);
  EXPECT_NEAR(p.wheel_drop, 0.05368f, 1e-4f);
  EXPECT_NEAR(p.wheel_x, 0.0712f, 1e-4f);
  EXPECT_NEAR(p.coupler_angle, -2.4496f, 1e-3f);
}

TEST(LegLinkage, ForwardMatchesCadAcrossRange)
{
  LegLinkage link(GAMMA);
  LegPose p;
  ASSERT_TRUE(link.Forward(static_cast<float>(M_PI / 6), p));
  EXPECT_NEAR(p.bar_angle, 0.2073f, 1e-3f);
  EXPECT_NEAR(p.wheel_drop, 0.06915f, 1e-4f);
  ASSERT_TRUE(link.Forward(static_cast<float>(M_PI / 3), p));
  EXPECT_NEAR(p.bar_angle, -0.1260f, 1e-3f);
  EXPECT_NEAR(p.wheel_drop, 0.08904f, 1e-4f);
  EXPECT_NEAR(p.coupler_angle, -1.7600f, 1e-3f);
  ASSERT_TRUE(link.Forward(static_cast<float>(M_PI / 2), p));
  EXPECT_NEAR(p.bar_angle, -0.4829f, 1e-3f);
  EXPECT_NEAR(p.wheel_drop, 0.10936f, 1e-4f);
  EXPECT_NEAR(p.coupler_angle, -1.2593f, 1e-3f);
}

TEST(LegLinkage, ForwardPicksLowerBranchEverywhere)
{
  LegLinkage link(GAMMA);
  LegPose p;
  for (float theta = -0.5f; theta <= 2.0f; theta += 0.05f) {
    if (link.Forward(theta, p)) {
      EXPECT_LT(p.bar_angle, 1.0f) << "theta " << theta;  // 上の枝は 1.45 付近（下の枝は θ = −0.5 でも 0.68）
    }
  }
}

TEST(LegLinkage, InverseRoundTrips)
{
  LegLinkage link(GAMMA);
  for (float theta = 0.0f; theta <= 1.5707963f; theta += 0.1f) {
    LegPose p;
    ASSERT_TRUE(link.Forward(theta, p));
    float back = -1.0f;
    ASSERT_TRUE(link.Inverse(p.wheel_drop, back)) << "theta " << theta;
    EXPECT_NEAR(back, theta, 1e-4f);
  }
}

TEST(LegLinkage, OutOfRangeReturnsFalseAndKeepsOutput)
{
  LegLinkage link(GAMMA);
  float theta = 0.3f;
  EXPECT_FALSE(link.Inverse(0.03f, theta));   // drop_min 0.05368 より浅い
  EXPECT_FALSE(link.Inverse(0.13f, theta));   // drop_max 0.10936 より深い
  EXPECT_FLOAT_EQ(theta, 0.3f);
  LegPose p = {1.0f, 2.0f, 3.0f, 4.0f};
  EXPECT_FALSE(link.Forward(3.3f, p));        // クランクが C（バーのピン + 連結リンク取り付け）の方を向き、円が内側で交わらない
  EXPECT_FLOAT_EQ(p.bar_angle, 1.0f);
}

TEST(LegLinkage, DropRangeAndJacobian)
{
  LegLinkage link(GAMMA);
  EXPECT_NEAR(link.drop_min(), 0.05368f, 1e-4f);
  EXPECT_NEAR(link.drop_max(), 0.10936f, 1e-4f);
  EXPECT_NEAR(link.DropJacobian(0.15f), 0.0273f, 5e-4f);
  EXPECT_NEAR(link.DropJacobian(0.8f), 0.0385f, 5e-4f);
  EXPECT_GT(link.DropJacobian(0.0f), 0.0f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
