#include <gtest/gtest.h>
#include "rabcl/controller/leg_damper.hpp"

namespace rabcl
{

class LegDamperTest : public ::testing::Test
{
protected:
  // kp 1, kd 0.1, load_ff 0.5, friction_ff 0.25, deadband 0.02, ramp_rate 0.5
  LegDamper damper_{LegDamperConfig{1.0f, 0.1f, 0.5f, 0.25f, 0.02f, 0.5f}};
};

TEST_F(LegDamperTest, ResetSetsRampedTarget)
{
  damper_.Reset(0.7f);
  EXPECT_FLOAT_EQ(damper_.ramped_target(), 0.7f);
  const MitCommand cmd = damper_.Update(0.7f, 0.7f, 0.0f, 0.01f);
  EXPECT_FLOAT_EQ(cmd.pos, 0.7f);
}

TEST_F(LegDamperTest, RampLimitsTargetSpeed)
{
  damper_.Reset(0.0f);
  MitCommand cmd = damper_.Update(1.0f, 0.0f, 0.0f, 0.1f);
  EXPECT_FLOAT_EQ(cmd.pos, 0.05f);  // 0.5 rad/s * 0.1 s
  for (int i = 0; i < 30; ++i) {
    cmd = damper_.Update(1.0f, 0.0f, 0.0f, 0.1f);
  }
  EXPECT_FLOAT_EQ(cmd.pos, 1.0f);  // 目標で止まる
  cmd = damper_.Update(0.9f, 0.0f, 0.0f, 0.1f);
  EXPECT_FLOAT_EQ(cmd.pos, 0.95f);  // 下る方向も同じ速さ
}

TEST_F(LegDamperTest, FrictionFfFollowsErrorSign)
{
  damper_.Reset(0.5f);
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.4f, 0.0f, 0.01f).tff, 0.75f);   // 目標が上 → +
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.6f, 0.0f, 0.01f).tff, 0.25f);   // 目標が下 → −
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.505f, 0.0f, 0.01f).tff, 0.5f);  // 不感帯の中
}

TEST_F(LegDamperTest, GainsPassThrough)
{
  damper_.Reset(0.3f);
  const MitCommand cmd = damper_.Update(0.3f, 0.3f, 0.0f, 0.01f);
  EXPECT_FLOAT_EQ(cmd.kp, 1.0f);
  EXPECT_FLOAT_EQ(cmd.kd, 0.1f);
  EXPECT_FLOAT_EQ(cmd.vel, 0.0f);
}

TEST_F(LegDamperTest, UpdateWithZeroDtHoldsTarget)
{
  damper_.Reset(0.2f);
  EXPECT_FLOAT_EQ(damper_.Update(1.0f, 0.2f, 0.0f, 0.0f).pos, 0.2f);
  EXPECT_FLOAT_EQ(damper_.Update(1.0f, 0.2f, 0.0f, -1.0f).pos, 0.2f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
