#include <gtest/gtest.h>
#include "rabcl/utils/utils.hpp"

namespace rabcl
{

TEST(UtilsTest, DegToRadReturnValue)
{
  EXPECT_DOUBLE_EQ(Utils::DegToRad(0.0), 0.0);
  EXPECT_DOUBLE_EQ(Utils::DegToRad(180.0), M_PI);
  EXPECT_DOUBLE_EQ(Utils::DegToRad(90.0), M_PI / 2.0);
  EXPECT_DOUBLE_EQ(Utils::DegToRad(-90.0), -M_PI / 2.0);
}

TEST(UtilsTest, DegToRadOutParam)
{
  double rad = 0.0;
  Utils::DegToRad(180.0, rad);
  EXPECT_DOUBLE_EQ(rad, M_PI);

  Utils::DegToRad(0.0, rad);
  EXPECT_DOUBLE_EQ(rad, 0.0);
}

TEST(UtilsTest, RadToDegReturnValue)
{
  EXPECT_DOUBLE_EQ(Utils::RadToDeg(0.0), 0.0);
  EXPECT_DOUBLE_EQ(Utils::RadToDeg(M_PI), 180.0);
  EXPECT_DOUBLE_EQ(Utils::RadToDeg(M_PI / 2.0), 90.0);
  EXPECT_DOUBLE_EQ(Utils::RadToDeg(-M_PI / 2.0), -90.0);
}

TEST(UtilsTest, RadToDegOutParam)
{
  double deg = 0.0;
  Utils::RadToDeg(M_PI, deg);
  EXPECT_DOUBLE_EQ(deg, 180.0);

  Utils::RadToDeg(0.0, deg);
  EXPECT_DOUBLE_EQ(deg, 0.0);
}

TEST(UtilsTest, ShortestPathNoChange)
{
  EXPECT_DOUBLE_EQ(Utils::ShortestPathMultiTurn(1.0, 1.0), 1.0);
}

TEST(UtilsTest, ShortestPathForward)
{
  EXPECT_NEAR(Utils::ShortestPathMultiTurn(1.0, 1.5), 1.5, 1e-9);
}

TEST(UtilsTest, ShortestPathBackward)
{
  EXPECT_NEAR(Utils::ShortestPathMultiTurn(1.0, 0.5), 0.5, 1e-9);
}

TEST(UtilsTest, ShortestPathWrapCW)
{
  // current=350°, target=10° (in rad) → should go +20° not -340°
  double current = 350.0 * M_PI / 180.0;
  double target = 10.0 * M_PI / 180.0;
  double result = Utils::ShortestPathMultiTurn(current, target);
  double expected = 370.0 * M_PI / 180.0;
  EXPECT_NEAR(result, expected, 1e-9);
}

TEST(UtilsTest, ShortestPathWrapCCW)
{
  // current=10°, target=350° → should go -20° not +340°
  double current = 10.0 * M_PI / 180.0;
  double target = 350.0 * M_PI / 180.0;
  double result = Utils::ShortestPathMultiTurn(current, target);
  double expected = -10.0 * M_PI / 180.0;
  EXPECT_NEAR(result, expected, 1e-9);
}

TEST(UtilsTest, ShortestPathAccumulate)
{
  double cmd = 4.0;
  cmd = Utils::ShortestPathMultiTurn(cmd, 4.5);
  EXPECT_NEAR(cmd, 4.5, 1e-9);
  cmd = Utils::ShortestPathMultiTurn(cmd, 0.2);  // wrap CW
  EXPECT_NEAR(cmd, 2.0 * M_PI + 0.2, 1e-9);
}

TEST(UtilsTest, ShortestPathCustomFullRotation)
{
  // full_rotation=36000 (0.01 deg units)
  double result = Utils::ShortestPathMultiTurn(35000.0, 1000.0, 36000.0);
  EXPECT_NEAR(result, 37000.0, 1e-9);
}

TEST(UtilsTest, WrapPi)
{
  const float pi = static_cast<float>(M_PI);
  EXPECT_NEAR(Utils::WrapPi(4.0f), 4.0f - 2.0f * pi, 1e-5f);
  EXPECT_NEAR(Utils::WrapPi(-4.0f), -4.0f + 2.0f * pi, 1e-5f);
  EXPECT_FLOAT_EQ(Utils::WrapPi(1.0f), 1.0f);
  EXPECT_LE(std::fabs(Utils::WrapPi(pi)), pi);
  EXPECT_NEAR(std::fabs(Utils::WrapPi(7.0f * pi)), pi, 1e-4f);
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
