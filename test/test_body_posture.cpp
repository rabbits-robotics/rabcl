#include <gtest/gtest.h>
#include <cmath>
#include "rabcl/controller/body_posture.hpp"

namespace rabcl
{

class BodyPostureTest : public ::testing::Test
{
protected:
  static constexpr float PI = static_cast<float>(M_PI);
  // gamma: 脚は対角（FR −45°, FL +45°, BR −135°, BL +135°）。gain √2 で 0.707 × √2 = 1 になり値が読みやすい
  BodyPosture posture_{BodyPostureConfig{
      0.15f, 1.45f, std::sqrt(2.0f), {-PI / 4.0f, PI / 4.0f, -3.0f * PI / 4.0f, 3.0f * PI / 4.0f}}};
  float out_[4] = {};

  void ExpectLegs(float fr, float fl, float br, float bl)
  {
    EXPECT_NEAR(out_[0], fr, 1e-5f) << "FR";
    EXPECT_NEAR(out_[1], fl, 1e-5f) << "FL";
    EXPECT_NEAR(out_[2], br, 1e-5f) << "BR";
    EXPECT_NEAR(out_[3], bl, 1e-5f) << "BL";
  }
};

TEST_F(BodyPostureTest, HeightMapsLinearly)
{
  posture_.Solve(0.0f, 0.0f, 0.0f, 0.0f, out_);
  ExpectLegs(0.15f, 0.15f, 0.15f, 0.15f);
  posture_.Solve(1.0f, 0.0f, 0.0f, 0.0f, out_);
  ExpectLegs(1.45f, 1.45f, 1.45f, 1.45f);
  posture_.Solve(0.5f, 0.0f, 0.0f, 0.0f, out_);
  ExpectLegs(0.80f, 0.80f, 0.80f, 0.80f);
}

TEST_F(BodyPostureTest, RollPositiveLowersRightSide)
{
  posture_.Solve(0.5f, 0.1f, 0.0f, 0.0f, out_);
  ExpectLegs(0.70f, 0.90f, 0.70f, 0.90f);
}

TEST_F(BodyPostureTest, PitchPositiveLowersFront)
{
  posture_.Solve(0.5f, 0.0f, 0.2f, 0.0f, out_);
  ExpectLegs(0.60f, 0.60f, 1.00f, 1.00f);
}

TEST_F(BodyPostureTest, YawOffsetRotatesTiltIntoChassisFrame)
{
  // 首が左 90° を向いているとき、カメラの「前が下がる」は車体の「左が下がる」
  posture_.Solve(0.5f, 0.0f, 0.2f, PI / 2.0f, out_);
  ExpectLegs(1.00f, 0.60f, 1.00f, 0.60f);
}

TEST_F(BodyPostureTest, YawOffsetPiFlipsSign)
{
  posture_.Solve(0.5f, 0.1f, 0.0f, PI, out_);
  ExpectLegs(0.90f, 0.70f, 0.90f, 0.70f);
}

TEST_F(BodyPostureTest, SolveClampsEveryLeg)
{
  posture_.Solve(2.0f, 5.0f, -5.0f, 0.3f, out_);
  for (float v : out_) {
    EXPECT_GE(v, 0.15f);
    EXPECT_LE(v, 1.45f);
  }
  posture_.Solve(-1.0f, 0.0f, 0.0f, 0.0f, out_);
  ExpectLegs(0.15f, 0.15f, 0.15f, 0.15f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
