#include <gtest/gtest.h>
#include "rabcl/controller/feeder.hpp"

namespace rabcl
{

namespace
{

// gamma: 4096 カウント / 8 セル / 増速 3 = 170.67 カウント / セル、200 ms 周期
Feeder MakeFeeder()
{
  return Feeder(FeederConfig{4096.0f / 24.0f, 0.2f});
}

using M = Feeder::Mode;

}  // namespace

TEST(Feeder, StopSendsNothing)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(0, f.Update(M::STOP, 1.0f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.5f));
  EXPECT_EQ(0, f.cells());
  EXPECT_EQ(0, f.sent_counts());
}

TEST(Feeder, FirstForwardStepsImmediately)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(1, f.cells());
  EXPECT_EQ(171, f.sent_counts());
  EXPECT_NEAR(170.667f, f.target_counts(), 0.01f);
}

TEST(Feeder, HeldRepeatsEveryPeriodWithoutAccumulatingFraction)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(0, f.Update(M::FORWARD, 1.1f));
  EXPECT_EQ(0, f.Update(M::FORWARD, 1.19f));
  EXPECT_EQ(170, f.Update(M::FORWARD, 1.2f));  // 341.33 → 341
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.4f));  // 512.00 → 512
  EXPECT_EQ(3, f.cells());
  EXPECT_EQ(512, f.sent_counts());
}

TEST(Feeder, TapGivesExactlyOneCell)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.05f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.3f));
  EXPECT_EQ(1, f.cells());
}

TEST(Feeder, RetapWithinPeriodIsHeldUntilPeriod)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.05f));
  EXPECT_EQ(0, f.Update(M::FORWARD, 1.1f));  // 保留
  EXPECT_EQ(0, f.Update(M::STOP, 1.15f));
  EXPECT_EQ(170, f.Update(M::STOP, 1.2f));   // 離していても保留の 1 発は出る
  EXPECT_EQ(0, f.Update(M::STOP, 1.4f));
  EXPECT_EQ(2, f.cells());
}

TEST(Feeder, RetapAfterPeriodStepsImmediately)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.05f));
  EXPECT_EQ(170, f.Update(M::FORWARD, 1.5f));
}

TEST(Feeder, ReverseStepsNegativeAndKeepsFraction)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(-171, f.Update(M::REVERSE, 1.0f));
  EXPECT_EQ(-170, f.Update(M::REVERSE, 1.2f));
  EXPECT_EQ(-171, f.Update(M::REVERSE, 1.4f));
  EXPECT_EQ(-3, f.cells());
  EXPECT_EQ(-512, f.sent_counts());
}

TEST(Feeder, SwitchingDirectionIsAnEdge)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(0, f.Update(M::REVERSE, 1.1f));     // 周期内なので保留
  EXPECT_EQ(-171, f.Update(M::REVERSE, 1.2f));  // 目標 0.33 → 0、送信済み 171 → −171
  EXPECT_EQ(0, f.cells());
  EXPECT_EQ(0, f.sent_counts());
}

TEST(Feeder, CancelDropsPendingCell)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.05f));
  EXPECT_EQ(0, f.Update(M::FORWARD, 1.1f));  // 保留
  f.Cancel();
  EXPECT_EQ(0, f.Update(M::STOP, 1.2f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.4f));
  EXPECT_EQ(1, f.cells());
}

TEST(Feeder, CancelDoesNotBlockNextEdge)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.05f));
  EXPECT_EQ(0, f.Update(M::FORWARD, 1.1f));
  f.Cancel();
  EXPECT_EQ(0, f.Update(M::STOP, 1.2f));
  EXPECT_EQ(170, f.Update(M::FORWARD, 2.0f));
  EXPECT_EQ(2, f.cells());
}

TEST(Feeder, HeldReverseAfterStopKeepsPeriodFromLastStep)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(171, f.Update(M::FORWARD, 1.0f));
  EXPECT_EQ(0, f.Update(M::STOP, 1.1f));
  EXPECT_EQ(0, f.Update(M::REVERSE, 1.15f));
  EXPECT_EQ(-171, f.Update(M::REVERSE, 1.2f));  // 目標 0 → 送信済み 171 との差
  EXPECT_EQ(-171, f.Update(M::REVERSE, 1.4f));  // 目標 −170.67 → −171
  EXPECT_EQ(-1, f.cells());
  EXPECT_EQ(-171, f.sent_counts());
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
