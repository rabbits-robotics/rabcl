#include <gtest/gtest.h>
#include "rabcl/controller/jam_recovery.hpp"

namespace rabcl
{

namespace
{

// gamma: 170.67 カウント / セル、負荷 80 %、2.0 A（308 × 6.5 mA）、1 回で検知、上限 2 セル、3 回で JAMMED、待ち 1 s、到達 ±5
JamRecoveryConfig MakeConfig()
{
  return JamRecoveryConfig{4096.0f / 24.0f, 800, 308, 1, 2, 3, 1.0f, 5};
}

JamRecoveryConfig MakeConfigTwoReads()
{
  JamRecoveryConfig c = MakeConfig();
  c.stall_reads = 2;
  return c;
}

using S = JamRecovery::State;

}  // namespace

TEST(JamRecoveryDetect, QuietReadKeepsNormal)
{
  JamRecovery r(MakeConfig());
  auto a = r.Observe(147, -140, 44, 0, 1.0f);  // 10 発/s の普段の読み
  EXPECT_EQ(S::NORMAL, r.state());
  EXPECT_FALSE(a.block_forward);
  EXPECT_FALSE(a.block_reverse);
  EXPECT_EQ(0, a.move);
  EXPECT_FALSE(a.reconnect);
}

TEST(JamRecoveryDetect, OneHighLoadReadBacksUp)
{
  JamRecovery r(MakeConfig());
  auto a = r.Observe(147, -1000, 545, 0, 1.0f);
  EXPECT_EQ(S::BACKING, r.state());
  EXPECT_EQ(-171, a.move);  // k = floor(147 / 170.67) + 1 = 1
  EXPECT_TRUE(a.block_forward);
}

TEST(JamRecoveryDetect, BackCountIsWholeCellsPastRemaining)
{
  JamRecovery r(MakeConfig());
  EXPECT_EQ(-512, r.Observe(490, -1000, 0, 0, 1.0f).move);  // k = 3、lround(3 × 170.67) = 512
}

TEST(JamRecoveryDetect, CurrentAloneDetects)
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -100, 308, 0, 1.0f);
  EXPECT_EQ(S::BACKING, r.state());
}

TEST(JamRecoveryDetect, LoadAtLimitDetectsBelowDoesNot)
{
  JamRecovery a(MakeConfig());
  a.Observe(147, -800, 0, 0, 1.0f);
  EXPECT_EQ(S::BACKING, a.state());
  JamRecovery b(MakeConfig());
  b.Observe(147, -799, 307, 0, 1.0f);
  EXPECT_EQ(S::NORMAL, b.state());
}

TEST(JamRecoveryDetect, ZeroRemainingNeverDetects)
{
  JamRecovery r(MakeConfig());
  r.Observe(0, -1000, 600, 0, 1.0f);
  EXPECT_EQ(S::NORMAL, r.state());
}

TEST(JamRecoveryDetect, TwoReadsConfigNeedsConsecutiveReads)
{
  JamRecovery r(MakeConfigTwoReads());
  r.Observe(147, -1000, 0, 0, 1.0f);
  EXPECT_EQ(S::NORMAL, r.state());
  r.Observe(147, -100, 0, 0, 1.1f);  // 途切れたら数え直し
  r.Observe(147, -1000, 0, 0, 1.2f);
  EXPECT_EQ(S::NORMAL, r.state());
  r.Observe(147, -1000, 0, 0, 1.3f);
  EXPECT_EQ(S::BACKING, r.state());
}

TEST(JamRecoveryDetect, ReverseStallBlocksReverseOnly)
{
  JamRecovery r(MakeConfig());
  auto a = r.Observe(-150, 1000, 500, 0, 1.0f);
  EXPECT_EQ(S::NORMAL, r.state());
  EXPECT_TRUE(a.block_reverse);
  EXPECT_FALSE(a.block_forward);
  EXPECT_EQ(0, a.move);
  a = r.Observe(-150, 60, 10, 0, 1.1f);  // 負荷が下がれば解除
  EXPECT_FALSE(a.block_reverse);
}

TEST(JamRecoveryQueue, ForwardBlockedAboveLimit)
{
  JamRecovery r(MakeConfig());
  EXPECT_FALSE(r.Observe(341, -100, 0, 0, 1.0f).block_forward);  // 2 セル = lround(341.33) ちょうどは通す
  EXPECT_TRUE(r.Observe(342, -100, 0, 0, 1.1f).block_forward);
  EXPECT_FALSE(r.Observe(342, -100, 0, 0, 1.1f).block_reverse);
  EXPECT_FALSE(r.Observe(170, -100, 0, 0, 1.2f).block_forward);
}

TEST(JamRecoveryQueue, ReverseBlockedBelowNegativeLimit)
{
  JamRecovery r(MakeConfig());
  auto a = r.Observe(-342, 100, 0, 0, 1.0f);
  EXPECT_TRUE(a.block_reverse);
  EXPECT_FALSE(a.block_forward);
}

namespace
{

// 進め直しを 1 回失敗させる: 戻り切った読み → 進め直しで詰まった読み
void FailRetry(JamRecovery & r, float t)
{
  r.Observe(0, -20, 0, 0, t);
  r.Observe(171, -1000, 545, 0, t + 0.1f);
}

}  // namespace

TEST(JamRecoveryCycle, BackThenRetryThenNormal)
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);         // → BACKING, move −171
  auto a = r.Observe(-120, 300, 50, 0, 1.1f);  // 戻り中（残りは負）
  EXPECT_EQ(S::BACKING, r.state());
  EXPECT_EQ(0, a.move);
  EXPECT_TRUE(a.block_forward);
  a = r.Observe(0, -20, 0, 0, 1.2f);  // 戻り切った
  EXPECT_EQ(S::RETRYING, r.state());
  EXPECT_EQ(171, a.move);
  EXPECT_TRUE(a.block_forward);
  a = r.Observe(90, -200, 30, 0, 1.3f);  // 進め直し中
  EXPECT_EQ(S::RETRYING, r.state());
  EXPECT_EQ(0, a.move);
  a = r.Observe(0, -20, 0, 0, 1.4f);
  EXPECT_EQ(S::NORMAL, r.state());
  EXPECT_FALSE(a.block_forward);
  EXPECT_EQ(1u, r.recoveries());
  EXPECT_EQ(0, r.tries());
}

TEST(JamRecoveryCycle, StallDuringRetryBacksAgainWithSameCounts)
{
  JamRecovery r(MakeConfig());
  r.Observe(490, -1000, 545, 0, 1.0f);          // k = 3, −512
  r.Observe(0, -20, 0, 0, 1.2f);                // → RETRYING +512
  auto a = r.Observe(512, -1000, 545, 0, 1.3f);  // 進まない
  EXPECT_EQ(S::BACKING, r.state());
  EXPECT_EQ(-512, a.move);  // 残り 512 から k を数え直すと 4 セルになる。同じ 512 を戻す
  EXPECT_EQ(1, r.tries());
}

TEST(JamRecoveryCycle, RetryTimeoutCountsAsFailure)
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);
  r.Observe(0, -20, 0, 0, 1.2f);       // → RETRYING at 1.2
  r.Observe(60, -300, 50, 0, 2.1f);    // 0.9 s: まだ待つ
  EXPECT_EQ(S::RETRYING, r.state());
  auto a = r.Observe(60, -300, 50, 0, 2.21f);  // 1.0 s 超え
  EXPECT_EQ(S::BACKING, r.state());
  EXPECT_EQ(-171, a.move);
  EXPECT_EQ(1, r.tries());
}

TEST(JamRecoveryCycle, BackingTimeoutReconnects)  // 実機 2026-11-21: 戻しても残り 92 のまま → JAMMED で押し続けて保護に落ちた
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);
  auto a = r.Observe(92, -728, 278, 0, 2.01f);
  EXPECT_EQ(S::RECONNECT, r.state());
  EXPECT_TRUE(a.reconnect);
  EXPECT_TRUE(a.block_forward);
  EXPECT_EQ(0, a.move);
  EXPECT_EQ(1u, r.jams());
}

TEST(JamRecoveryGiveUp, JammedNotArrivedReconnects)  // 最後の戻しも効かないなら引っ張り続けない
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);
  FailRetry(r, 1.2f);
  FailRetry(r, 1.5f);
  FailRetry(r, 1.8f);  // 1.9 で JAMMED、−171 を送った
  ASSERT_EQ(S::JAMMED, r.state());
  r.Observe(100, -700, 200, 0, 2.5f);
  EXPECT_EQ(S::JAMMED, r.state());
  auto a = r.Observe(100, -700, 200, 0, 2.95f);
  EXPECT_EQ(S::RECONNECT, r.state());
  EXPECT_TRUE(a.reconnect);
}


TEST(JamRecoveryGiveUp, ThirdFailureBacksOffThenJams)
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);  // 検知、戻し
  FailRetry(r, 1.2f);
  EXPECT_EQ(1, r.tries());
  EXPECT_EQ(S::BACKING, r.state());
  FailRetry(r, 1.5f);
  EXPECT_EQ(2, r.tries());
  r.Observe(0, -20, 0, 0, 1.8f);                 // → RETRYING 3 回目
  auto a = r.Observe(171, -1000, 545, 0, 1.9f);  // 3 回目の失敗
  EXPECT_EQ(S::JAMMED, r.state());
  EXPECT_EQ(-171, a.move);  // 最後も戻してから JAMMED
  EXPECT_TRUE(a.block_forward);
  EXPECT_EQ(1u, r.jams());
  EXPECT_EQ(3, r.tries());
  a = r.Observe(0, -20, 0, 0, 2.5f);  // JAMMED は残り 0 でも抜けない
  EXPECT_EQ(S::JAMMED, r.state());
  EXPECT_TRUE(a.block_forward);
  EXPECT_FALSE(a.block_reverse);
}

TEST(JamRecoveryGiveUp, ReverseClearsJammed)
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);
  FailRetry(r, 1.2f);
  FailRetry(r, 1.5f);
  FailRetry(r, 1.8f);
  ASSERT_EQ(S::JAMMED, r.state());
  r.NotifyReverse();
  EXPECT_EQ(S::NORMAL, r.state());
  EXPECT_EQ(0, r.tries());
  EXPECT_FALSE(r.Observe(-171, 100, 20, 0, 2.0f).block_forward);
}

TEST(JamRecoveryOperator, ReverseDuringRecoveryAbortsToNormal)
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);  // BACKING
  r.NotifyReverse();
  EXPECT_EQ(S::NORMAL, r.state());
  auto a = r.Observe(-200, 100, 20, 0, 1.1f);  // 残りが負でも queue ルールだけ
  EXPECT_FALSE(a.block_forward);
  EXPECT_FALSE(a.block_reverse);
  EXPECT_EQ(0, a.move);
  r.Observe(147, -1000, 545, 0, 2.0f);
  r.Observe(0, -20, 0, 0, 2.2f);  // RETRYING
  r.NotifyReverse();
  EXPECT_EQ(S::NORMAL, r.state());
  EXPECT_EQ(0, r.tries());
}

TEST(JamRecoveryProtect, OverloadFlagRequestsReconnectFromAnyState)
{
  JamRecovery r(MakeConfig());
  auto a = r.Observe(2410, -200, 20, 0x20, 1.0f);  // NORMAL で保護
  EXPECT_TRUE(a.reconnect);
  EXPECT_TRUE(a.block_forward);
  EXPECT_EQ(0, a.move);
  EXPECT_EQ(S::RECONNECT, r.state());
  a = r.Observe(0, -20, 0, 0, 1.1f);  // Reset まで留まる
  EXPECT_EQ(S::RECONNECT, r.state());
  EXPECT_TRUE(a.block_forward);
  r.Reset();
  EXPECT_EQ(S::NORMAL, r.state());
  EXPECT_EQ(0, r.tries());
  JamRecovery s(MakeConfig());
  s.Observe(147, -1000, 545, 0, 1.0f);  // BACKING で保護
  EXPECT_TRUE(s.Observe(-171, -200, 20, 0x20, 1.1f).reconnect);
  EXPECT_EQ(S::RECONNECT, s.state());
}

TEST(JamRecoveryDetect, DetectsWithoutAnySend)  // L1 を離したあとの詰まり: 入力は残りと負荷だけ
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -100, 0, 0, 1.0f);
  r.Observe(147, -1000, 545, 0, 1.1f);
  EXPECT_EQ(S::BACKING, r.state());
}

TEST(JamRecoveryCycle, SmallResidualCountsAsArrived)  // 静止時の残りは 0 にならず ±3 残る
{
  JamRecovery r(MakeConfig());
  r.Observe(147, -1000, 545, 0, 1.0f);
  auto a = r.Observe(-2, -20, 0, 0, 1.2f);  // 戻り切った扱い
  EXPECT_EQ(S::RETRYING, r.state());
  EXPECT_EQ(171, a.move);
  a = r.Observe(5, -20, 0, 0, 1.4f);  // ちょうど許容
  EXPECT_EQ(S::NORMAL, r.state());
  EXPECT_EQ(1u, r.recoveries());
  JamRecovery q(MakeConfig());
  q.Observe(3, -1000, 600, 0, 1.0f);  // 残りが許容内なら高負荷でも詰まりではない
  EXPECT_EQ(S::NORMAL, q.state());
}

namespace
{

// 戻しが効かずに再接続へ: 検知 → 戻しのタイムアウト
void JamToReconnect(JamRecovery & r, float t)
{
  r.Observe(147, -1000, 545, 0, t);
  r.Observe(140, -900, 400, 0, t + 1.01f);
}

}  // namespace

TEST(JamRecoveryReconnect, ThirdJamReconnectResumesJammed)  // 実機 2026-11-21: 再接続 → 位相合わせで噛む → 再接続 … が 3 秒ごとに続いた
{
  JamRecovery r(MakeConfig());
  JamToReconnect(r, 1.0f);
  ASSERT_EQ(S::RECONNECT, r.state());
  EXPECT_EQ(1, r.tries());
  r.ResumeAfterReconnect(3.0f);
  EXPECT_EQ(S::NORMAL, r.state());
  JamToReconnect(r, 4.0f);
  EXPECT_EQ(2, r.tries());
  r.ResumeAfterReconnect(6.0f);
  EXPECT_EQ(S::NORMAL, r.state());
  JamToReconnect(r, 7.0f);
  EXPECT_EQ(3, r.tries());
  r.ResumeAfterReconnect(9.0f);
  EXPECT_EQ(S::JAMMED, r.state());
  auto a = r.Observe(-150, 300, 40, 0, 9.01f);  // 後ろ合わせの Move がまだ動いている: 再接続に落ちない
  EXPECT_EQ(S::JAMMED, r.state());
  EXPECT_FALSE(a.reconnect);
  EXPECT_TRUE(r.Observe(2, -10, 0, 0, 10.0f).block_forward);
  r.NotifyReverse();
  EXPECT_EQ(S::NORMAL, r.state());
  EXPECT_EQ(0, r.tries());
}

TEST(JamRecoveryReconnect, HealthyCellResetsTries)
{
  JamRecovery r(MakeConfig());
  JamToReconnect(r, 1.0f);
  r.ResumeAfterReconnect(2.5f);
  EXPECT_EQ(1, r.tries());
  r.Observe(2, -10, 0, 0, 3.0f);  // 合わせ直した穴で静止しているだけでは数えない
  EXPECT_EQ(1, r.tries());
  r.Observe(147, -100, 10, 0, 3.1f);  // 1 セル送って動いている
  r.Observe(3, -10, 0, 0, 3.2f);      // 詰まらずに到達
  EXPECT_EQ(0, r.tries());
}

TEST(JamRecoveryReconnect, ProtectionReconnectAlsoCounts)
{
  JamRecovery r(MakeConfig());
  r.Observe(2410, -200, 20, 0x20, 1.0f);
  EXPECT_EQ(1, r.tries());
  r.Reset();  // 通信途絶からの再接続は数え直し
  EXPECT_EQ(0, r.tries());
}

}  // namespace rabcl
