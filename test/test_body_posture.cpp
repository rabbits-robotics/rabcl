#include <gtest/gtest.h>

#include <cmath>

#include "rabcl/controller/body_posture.hpp"

namespace rabcl
{

class BodyPostureTest : public ::testing::Test
{
protected:
  static constexpr float PI = static_cast<float>(M_PI);
  // gamma の寸法（設計書 2026-10-25-gamma-leg-linkage-design.md）。脚は対角（FR −45°, FL +45°, BR −135°, BL +135°）
  static constexpr LegLinkageConfig LINKAGE = {
    0.041f, 1.0471976f, 0.03975f, 0.060f, -0.026f, -0.049f, 0.0f, 0.0175f, 0.044f, -0.0325f,
    1.5707963f};
  static constexpr float DROP_MIN = 0.05756f;  // θ 0.15 rad
  static constexpr float DROP_MAX = 0.10377f;  // θ 1.42 rad
  BodyPosture posture_{BodyPostureConfig{
      LINKAGE, DROP_MIN, DROP_MAX, 0.170f,
      {-PI / 4.0f, PI / 4.0f, -3.0f * PI / 4.0f, 3.0f * PI / 4.0f}}};
  float out_[4] = {};

  // 出力の θ を車輪の下がりに戻して比べる
  float Drop(int i) const
  {
    LegPose p;
    EXPECT_TRUE(posture_.linkage().Forward(out_[i], p));
    return p.wheel_drop;
  }

  void ExpectDrops(float fr, float fl, float br, float bl)
  {
    EXPECT_NEAR(Drop(0), fr, 1e-4f) << "FR";
    EXPECT_NEAR(Drop(1), fl, 1e-4f) << "FL";
    EXPECT_NEAR(Drop(2), br, 1e-4f) << "BR";
    EXPECT_NEAR(Drop(3), bl, 1e-4f) << "BL";
  }
};

TEST_F(BodyPostureTest, HeightMapsToDropLinearly)
{
  posture_.Solve(0.0f, 0.0f, 0.0f, 0.0f, out_);
  for (float v : out_) {
    EXPECT_NEAR(v, 0.15f, 1e-3f);
  }
  posture_.Solve(1.0f, 0.0f, 0.0f, 0.0f, out_);
  for (float v : out_) {
    EXPECT_NEAR(v, 1.42f, 1e-3f);
  }
  posture_.Solve(0.5f, 0.0f, 0.0f, 0.0f, out_);
  ExpectDrops(0.080665f, 0.080665f, 0.080665f, 0.080665f);
}

TEST_F(BodyPostureTest, RollPositiveLowersRightSide)
{
  // 傾き 0.1 rad × 半径 0.170 × sin 45° = 0.012021 の差（右が浅い = 右が下がる）
  posture_.Solve(0.5f, 0.1f, 0.0f, 0.0f, out_);
  ExpectDrops(0.068644f, 0.092686f, 0.068644f, 0.092686f);
}

TEST_F(BodyPostureTest, PitchPositiveLowersFront)
{
  posture_.Solve(0.5f, 0.0f, 0.1f, 0.0f, out_);
  ExpectDrops(0.068644f, 0.068644f, 0.092686f, 0.092686f);
}

TEST_F(BodyPostureTest, YawOffsetRotatesTiltIntoChassisFrame)
{
  // 首が左 90° を向いているとき、カメラの「前が下がる」は車体の「左が下がる」
  posture_.Solve(0.5f, 0.0f, 0.1f, PI / 2.0f, out_);
  ExpectDrops(0.092686f, 0.068644f, 0.092686f, 0.068644f);
}

TEST_F(BodyPostureTest, YawOffsetPiFlipsSign)
{
  posture_.Solve(0.5f, 0.1f, 0.0f, PI, out_);
  ExpectDrops(0.092686f, 0.068644f, 0.092686f, 0.068644f);
}

TEST_F(BodyPostureTest, DropRangeOutsideLinkageFallsBackToMechanicalLimits)
{
  // 設定の drop_max がリンクの範囲（0.10936）を超えていても、車高 1 は最低車高にならず θmax 側に寄る
  BodyPosture wide{BodyPostureConfig{
      LINKAGE, 0.03f, 0.12f, 0.170f,
      {-PI / 4.0f, PI / 4.0f, -3.0f * PI / 4.0f, 3.0f * PI / 4.0f}}};
  float out[4] = {};
  wide.Solve(1.0f, 0.0f, 0.0f, 0.0f, out);
  for (float v : out) {
    EXPECT_NEAR(v, 1.5707963f, 1e-3f);
  }
  wide.Solve(0.0f, 0.0f, 0.0f, 0.0f, out);
  for (float v : out) {
    EXPECT_NEAR(v, 0.0f, 1e-3f);
  }
}

TEST_F(BodyPostureTest, SolveClampsEveryLeg)
{
  posture_.Solve(2.0f, 5.0f, -5.0f, 0.3f, out_);
  for (float v : out_) {
    EXPECT_GE(v, 0.15f - 1e-3f);
    EXPECT_LE(v, 1.42f + 1e-3f);
  }
  posture_.Solve(-1.0f, 0.0f, 0.0f, 0.0f, out_);
  for (float v : out_) {
    EXPECT_NEAR(v, 0.15f, 1e-3f);
  }
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
