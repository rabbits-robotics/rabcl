#include <gtest/gtest.h>
#include "rabcl/controller/leg_damper.hpp"

namespace rabcl
{

class LegDamperTest : public ::testing::Test
{
protected:
  // kp 1, kd 0.1, friction_ff 0.25, friction_band 0.04, ramp_rate 0.5。荷重分は Update の引数（テストでは 0.5）
  LegDamper damper_{LegDamperConfig{1.0f, 0.1f, 0.25f, 0.04f, 0.5f}};
};

TEST_F(LegDamperTest, ResetSetsRampedTarget)
{
  damper_.Reset(0.7f);
  EXPECT_FLOAT_EQ(damper_.ramped_target(), 0.7f);
  const MitCommand cmd = damper_.Update(0.7f, 0.7f, 0.0f, 0.01f, 0.5f);
  EXPECT_FLOAT_EQ(cmd.pos, 0.7f);
}

TEST_F(LegDamperTest, RampLimitsTargetSpeed)
{
  damper_.Reset(0.0f);
  MitCommand cmd = damper_.Update(1.0f, 0.0f, 0.0f, 0.1f, 0.5f);
  EXPECT_FLOAT_EQ(cmd.pos, 0.05f);  // 0.5 rad/s * 0.1 s
  for (int i = 0; i < 30; ++i) {
    cmd = damper_.Update(1.0f, 0.0f, 0.0f, 0.1f, 0.5f);
  }
  EXPECT_FLOAT_EQ(cmd.pos, 1.0f);  // 目標で止まる
  cmd = damper_.Update(0.9f, 0.0f, 0.0f, 0.1f, 0.5f);
  EXPECT_FLOAT_EQ(cmd.pos, 0.95f);  // 下る方向も同じ速さ
}

TEST_F(LegDamperTest, FrictionFfSaturatesOutsideBand)
{
  damper_.Reset(0.5f);
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.4f, 0.0f, 0.01f, 0.5f).tff, 0.75f);  // 目標が上（差 0.1 > band）→ +0.25
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.6f, 0.0f, 0.01f, 0.5f).tff, 0.25f);  // 目標が下 → −0.25
}

// band の中は差に比例して 0 までつながる（if の境界で出力が跳ばない）
TEST_F(LegDamperTest, FrictionFfIsLinearInsideBand)
{
  damper_.Reset(0.5f);
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.5f, 0.0f, 0.01f, 0.5f).tff, 0.5f);       // 差 0 → 0
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.49f, 0.0f, 0.01f, 0.5f).tff, 0.5625f);   // 差 +0.01 → +0.25 × 0.25
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.48f, 0.0f, 0.01f, 0.5f).tff, 0.625f);    // 差 +0.02 → +0.125
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.52f, 0.0f, 0.01f, 0.5f).tff, 0.375f);    // 差 −0.02 → −0.125
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.46f, 0.0f, 0.01f, 0.5f).tff, 0.75f);     // 差 +0.04 = band の縁 → +0.25（外と連続）
}

TEST_F(LegDamperTest, FrictionFfWithZeroBandFallsBackToSign)
{
  LegDamper d{LegDamperConfig{1.0f, 0.1f, 0.25f, 0.0f, 0.5f}};
  d.Reset(0.5f);
  EXPECT_FLOAT_EQ(d.Update(0.5f, 0.499f, 0.0f, 0.01f, 0.0f).tff, 0.25f);
  EXPECT_FLOAT_EQ(d.Update(0.5f, 0.501f, 0.0f, 0.01f, 0.0f).tff, -0.25f);
  EXPECT_FLOAT_EQ(d.Update(0.5f, 0.5f, 0.0f, 0.01f, 0.0f).tff, 0.0f);
}

TEST_F(LegDamperTest, ZeroFrictionFfAndZeroBandGiveNoNan)
{
  LegDamper d{LegDamperConfig{0.0f, 0.0f, 0.0f, 0.0f, 0.5f}};  // 偽ファームの設定（ランプだけ使う）
  d.Reset(0.5f);
  EXPECT_FLOAT_EQ(d.Update(0.5f, 0.4f, 0.0f, 0.01f, 0.0f).tff, 0.0f);
}

TEST_F(LegDamperTest, GainsPassThrough)
{
  damper_.Reset(0.3f);
  const MitCommand cmd = damper_.Update(0.3f, 0.3f, 0.0f, 0.01f, 0.5f);
  EXPECT_FLOAT_EQ(cmd.kp, 1.0f);
  EXPECT_FLOAT_EQ(cmd.kd, 0.1f);
  EXPECT_FLOAT_EQ(cmd.vel, 0.0f);
}

TEST_F(LegDamperTest, LoadTorqueAddsToTff)
{
  damper_.Reset(0.5f);
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.5f, 0.0f, 0.01f, 1.25f).tff, 1.25f);  // 差 0: 荷重分だけ
  EXPECT_FLOAT_EQ(damper_.Update(0.5f, 0.4f, 0.0f, 0.01f, 1.25f).tff, 1.5f);   // + 摩擦分
}

TEST_F(LegDamperTest, UpdateWithZeroDtHoldsTarget)
{
  damper_.Reset(0.2f);
  EXPECT_FLOAT_EQ(damper_.Update(1.0f, 0.2f, 0.0f, 0.0f, 0.5f).pos, 0.2f);
  EXPECT_FLOAT_EQ(damper_.Update(1.0f, 0.2f, 0.0f, -1.0f, 0.5f).pos, 0.2f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
