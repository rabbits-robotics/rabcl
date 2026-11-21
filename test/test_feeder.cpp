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

// 起動時の位相合わせ: 絶対角（位置モードで読む）から一番近い穴までの相対ステップ。
// gamma の実測（2026-11-21）: 穴の位置 = 30 + k × 170.67
TEST(FeederAlign, ZeroAtHole)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(0, f.AlignSteps(30, 30.0f));
  EXPECT_EQ(0, f.AlignSteps(542, 30.0f));  // 30 + 3 × 170.67 = 542.0
}

TEST(FeederAlign, BacksUpWhenJustPastHole)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(-20, f.AlignSteps(562, 30.0f));
}

TEST(FeederAlign, GoesForwardWhenJustBeforeHole)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(20, f.AlignSteps(522, 30.0f));
}

TEST(FeederAlign, HalfCellGoesForward)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(-85, f.AlignSteps(627, 30.0f));  // 位相 85.0 < 85.33: 戻る
  EXPECT_EQ(85, f.AlignSteps(628, 30.0f));   // 位相 86.0 ≥ 85.33: 進む（170.67 − 86 = 84.67 → 85）
}

TEST(FeederAlign, WorksBeyondOneTurnAndForNegativePositions)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(1, f.AlignSteps(4296, 30.0f));    // 実測 3 点目。位相 170.0 → 進んで 0.67 → 1
  EXPECT_EQ(4, f.AlignSteps(3951, 30.0f));    // 実測 1 点目。位相 166.33 → 4.33 → 4
  EXPECT_EQ(-41, f.AlignSteps(-100, 30.0f));  // −130 mod 170.67 = 40.67 → 戻る −41
}

TEST(FeederAlign, OffsetOutsideOneCellIsWrapped)
{
  Feeder f = MakeFeeder();
  EXPECT_EQ(0, f.AlignSteps(30, 30.0f + 4096.0f / 24.0f));
  EXPECT_EQ(0, f.AlignSteps(30, 30.0f - 4096.0f / 24.0f));
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
