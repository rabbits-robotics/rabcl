#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <utility>
#include <vector>
#include "rabcl/component/bno055.hpp"
#include "rabcl/utils/utils.hpp"

namespace rabcl
{

// --- BNO055Uart: フレームの組み立てと受信パーサ ---

TEST(BNO055Uart, BuildReadFrame)
{
  uint8_t out[8] = {};
  ASSERT_EQ(BNO055Uart::BuildRead(0x14, 12, out), 4u);
  const uint8_t expected[4] = {0xAA, 0x01, 0x14, 0x0C};
  EXPECT_EQ(std::memcmp(out, expected, 4), 0);
}

TEST(BNO055Uart, BuildWriteFrame)
{
  uint8_t out[8] = {};
  const uint8_t data[1] = {0x08};
  ASSERT_EQ(BNO055Uart::BuildWrite(0x3D, data, 1, out), 5u);
  const uint8_t expected[5] = {0xAA, 0x00, 0x3D, 0x01, 0x08};
  EXPECT_EQ(std::memcmp(out, expected, 5), 0);
}

TEST(BNO055Uart, PushDataResponse)
{
  BNO055Uart uart;
  BNO055Uart::Response r = {};
  const uint8_t frame[14] = {0xBB, 0x0C, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  for (size_t i = 0; i < 13; ++i) {
    EXPECT_FALSE(uart.Push(frame[i], r)) << "byte " << i;
  }
  ASSERT_TRUE(uart.Push(frame[13], r));
  EXPECT_TRUE(r.is_data);
  EXPECT_EQ(r.len, 12);
  EXPECT_EQ(std::memcmp(r.data, &frame[2], 12), 0);
}

TEST(BNO055Uart, PushAckResponse)
{
  BNO055Uart uart;
  BNO055Uart::Response r = {};
  EXPECT_FALSE(uart.Push(0xEE, r));
  ASSERT_TRUE(uart.Push(0x01, r));
  EXPECT_FALSE(r.is_data);
  EXPECT_EQ(r.status, BNO055Uart::Status::WRITE_SUCCESS);
}

TEST(BNO055Uart, PushSkipsLeadingGarbage)
{
  BNO055Uart uart;
  BNO055Uart::Response r = {};
  const uint8_t bytes[5] = {0x12, 0x34, 0xBB, 0x01, 0xA0};
  bool got = false;
  for (uint8_t b : bytes) {
    got = uart.Push(b, r);
  }
  ASSERT_TRUE(got);
  EXPECT_TRUE(r.is_data);
  EXPECT_EQ(r.len, 1);
  EXPECT_EQ(r.data[0], 0xA0);
}

TEST(BNO055Uart, PushRejectsBadLength)
{
  BNO055Uart uart;
  BNO055Uart::Response r = {};
  // len 0 は捨てて先頭探しに戻る
  EXPECT_FALSE(uart.Push(0xBB, r));
  EXPECT_FALSE(uart.Push(0x00, r));
  EXPECT_FALSE(uart.Push(0xBB, r));
  EXPECT_FALSE(uart.Push(0x01, r));
  ASSERT_TRUE(uart.Push(0xA0, r));
  EXPECT_EQ(r.data[0], 0xA0);
  // len 129 も同様
  EXPECT_FALSE(uart.Push(0xBB, r));
  EXPECT_FALSE(uart.Push(0x81, r));
  EXPECT_FALSE(uart.Push(0xEE, r));
  ASSERT_TRUE(uart.Push(0x07, r));
  EXPECT_FALSE(r.is_data);
  EXPECT_EQ(r.status, BNO055Uart::Status::BUS_OVER_RUN_ERROR);
}

TEST(BNO055Uart, PushByteByByte)
{
  BNO055Uart uart;
  BNO055Uart::Response r = {};
  std::vector<uint8_t> frame = {0xBB, 0x0C};
  for (uint8_t i = 0; i < 12; ++i) {
    frame.push_back(static_cast<uint8_t>(0x10 + i));
  }
  int trues = 0;
  for (size_t i = 0; i < frame.size(); ++i) {
    if (uart.Push(frame[i], r)) {
      ++trues;
      EXPECT_EQ(i, frame.size() - 1);
    }
  }
  EXPECT_EQ(trues, 1);
  EXPECT_EQ(r.data[11], 0x1B);
}

TEST(BNO055Uart, ResetDropsPartialFrame)
{
  BNO055Uart uart;
  BNO055Uart::Response r = {};
  EXPECT_FALSE(uart.Push(0xBB, r));
  EXPECT_FALSE(uart.Push(0x0C, r));
  EXPECT_FALSE(uart.Push(0x01, r));
  uart.Reset();
  EXPECT_FALSE(uart.Push(0xEE, r));
  ASSERT_TRUE(uart.Push(0x01, r));
  EXPECT_FALSE(r.is_data);
  EXPECT_EQ(r.status, BNO055Uart::Status::WRITE_SUCCESS);
}


// --- BNO055Sequencer: バスに依らない状態機械 ---

class BNO055SequencerTest : public ::testing::Test
{
protected:
  using Op = BNO055Op;
  using State = BNO055Sequencer::State;

  static BNO055Config DefaultConfig()
  {
    BNO055Config c;  // 既定値
    return c;
  }

  // 台本: 操作に対する結果を作る（CHIP_ID → A0、SYS_STATUS → 5、CLK_STATUS → 0、書き → ok、データ読み → data_）
  BNO055Result Answer(const Op & op)
  {
    BNO055Result r = {};
    r.ok = true;
    if (op.kind == Op::Kind::WRITE) {
      return r;
    }
    r.len = op.len;
    if (op.reg == 0x00) {
      r.data[0] = chip_id_;
    } else if (op.reg == 0x39) {
      r.data[0] = sys_status_;
    } else if (op.reg == 0x3A) {
      r.data[0] = sys_err_;
    } else if (op.reg == 0x38) {
      r.data[0] = 0x00;
    } else if (op.reg == 0x35) {
      r.data[0] = calib_;
    } else if (op.reg == 0x14) {
      std::memcpy(r.data, data_, 12);
    }
    return r;
  }

  // from_ms から to_ms まで step_ms 刻みで Next → 台本の結果を Complete。出た操作を ops_ に記録
  int Run(BNO055Sequencer & seq, uint32_t from_ms, uint32_t to_ms, uint32_t step_ms = 1)
  {
    int samples = 0;
    for (uint32_t t = from_ms; t <= to_ms; t += step_ms) {
      Op op = {};
      if (!seq.Next(t, op)) {
        continue;
      }
      ops_.push_back(op);
      if (!op.expect_response) {
        BNO055Result r = {};
        r.ok = true;
        seq.Complete(r, t, sample_);
        continue;
      }
      if (!answer_) {
        continue;  // 応答しない（タイムアウトさせる）
      }
      if (fail_error_ != 0) {
        BNO055Result r = {};
        r.ok = false;
        r.error = fail_error_;
        if (seq.Complete(r, t, sample_)) {
          ++samples;
        }
        continue;
      }
      if (seq.Complete(Answer(op), t, sample_)) {
        ++samples;
      }
    }
    return samples;
  }

  static void ExpectWrite(const Op & op, uint8_t reg, uint8_t value)
  {
    EXPECT_EQ(op.kind, Op::Kind::WRITE);
    EXPECT_EQ(op.reg, reg);
    EXPECT_EQ(op.len, 1);
    EXPECT_EQ(op.data[0], value);
    EXPECT_TRUE(op.expect_response);
  }

  static void ExpectRead(const Op & op, uint8_t reg, uint8_t len)
  {
    EXPECT_EQ(op.kind, Op::Kind::READ);
    EXPECT_EQ(op.reg, reg);
    EXPECT_EQ(op.len, len);
    EXPECT_TRUE(op.expect_response);
  }

  int CountRead(uint8_t reg) const
  {
    int n = 0;
    for (const auto & op : ops_) {
      if (op.kind == Op::Kind::READ && op.reg == reg) {
        ++n;
      }
    }
    return n;
  }

  std::vector<Op> ops_;
  BNO055Sample sample_ = {};
  uint8_t chip_id_ = 0xA0;
  uint8_t sys_status_ = 5;
  uint8_t sys_err_ = 0;
  uint8_t calib_ = 0xF0;
  uint8_t data_[12] = {};
  bool answer_ = true;
  uint8_t fail_error_ = 0;
};

TEST_F(BNO055SequencerTest, NoOpBeforeBootWait)
{
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 699);
  EXPECT_TRUE(ops_.empty());
  Run(seq, 700, 700);
  ASSERT_EQ(ops_.size(), 1u);
  ExpectRead(ops_[0], 0x00, 1);
}

TEST_F(BNO055SequencerTest, BootSequenceOpsInOrder)
{
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 800);
  ASSERT_GE(ops_.size(), 8u);
  ExpectRead(ops_[0], 0x00, 1);
  ExpectWrite(ops_[1], 0x3D, 0x00);
  ExpectWrite(ops_[2], 0x07, 0x00);
  ExpectWrite(ops_[3], 0x3B, 0x06);
  ExpectWrite(ops_[4], 0x41, 0x24);
  ExpectWrite(ops_[5], 0x42, 0x00);
  ExpectWrite(ops_[6], 0x3D, 0x08);
  ExpectRead(ops_[7], 0x39, 1);
  EXPECT_TRUE(seq.running());
  EXPECT_EQ(seq.state(), State::RUNNING);
  EXPECT_EQ(CountRead(0x38), 0);
  EXPECT_EQ(seq.error_count(), 0u);
}

TEST_F(BNO055SequencerTest, ExternalCrystalAddsClockSteps)
{
  BNO055Config c = DefaultConfig();
  c.use_external_crystal = true;
  BNO055Sequencer seq(c);
  Run(seq, 0, 900);
  ASSERT_GE(ops_.size(), 10u);
  ExpectWrite(ops_[5], 0x42, 0x00);
  ExpectRead(ops_[6], 0x38, 1);
  ExpectWrite(ops_[7], 0x3F, 0x80);
  ExpectWrite(ops_[8], 0x3D, 0x08);
  EXPECT_TRUE(seq.running());
}

TEST_F(BNO055SequencerTest, ModeSwitchWaits)
{
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 701);  // CHECK_ID → TO_CONFIG の書き（701 ms で応答済み）
  ASSERT_EQ(ops_.size(), 2u);
  ExpectWrite(ops_[1], 0x3D, 0x00);
  Run(seq, 702, 719);  // 19 ms 待ち
  EXPECT_EQ(ops_.size(), 2u);
  Run(seq, 720, 725);
  ASSERT_GE(ops_.size(), 3u);  // 720 ms で待ちが明け、以降は 1 ms ごとに次の段へ進む
  ExpectWrite(ops_[2], 0x07, 0x00);
}

TEST_F(BNO055SequencerTest, RunningReadsEvery10ms)
{
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 800);
  ASSERT_TRUE(seq.running());
  ops_.clear();
  Run(seq, 801, 1800);  // 1 s
  EXPECT_EQ(CountRead(0x14), 99);
  EXPECT_EQ(CountRead(0x35), 1);
  for (const auto & op : ops_) {
    if (op.reg == 0x14) {
      EXPECT_EQ(op.len, 12);
    }
  }
}

TEST_F(BNO055SequencerTest, SampleIsScaled)
{
  // gyro z = 900 → 1.0 rad/s、heading = 2827 → π
  data_[4] = 900 & 0xFF;
  data_[5] = 900 >> 8;
  data_[6] = 2827 & 0xFF;
  data_[7] = 2827 >> 8;
  calib_ = 0xF0;
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 800);
  const int samples = Run(seq, 801, 1800);
  EXPECT_EQ(samples, 99);
  EXPECT_FLOAT_EQ(sample_.gyro[2], 1.0f);
  EXPECT_NEAR(sample_.euler[0], static_cast<float>(M_PI), 1e-3f);
  EXPECT_EQ(seq.calib_stat(), 0xF0);
  EXPECT_EQ(sample_.calib_stat, 0xF0);
}

TEST_F(BNO055SequencerTest, TimeoutReissuesAndCountsError)
{
  BNO055Sequencer seq(DefaultConfig());
  answer_ = false;
  Run(seq, 0, 700);  // CHECK_ID を出す（応答なし）
  ASSERT_EQ(ops_.size(), 1u);
  Run(seq, 701, 719);
  EXPECT_EQ(ops_.size(), 1u);  // 20 ms 未満は待つ
  EXPECT_TRUE(seq.waiting());
  Run(seq, 720, 721);
  EXPECT_EQ(ops_.size(), 2u);  // 再発行
  ExpectRead(ops_[1], 0x00, 1);
  EXPECT_EQ(seq.error_count(), 1u);
  answer_ = true;
  Run(seq, 722, 800);
  EXPECT_EQ(seq.error_count(), 0u);
  EXPECT_TRUE(seq.running());
}

TEST_F(BNO055SequencerTest, BusOverrunRetriesWithoutReinit)
{
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 800);
  ASSERT_TRUE(seq.running());
  fail_error_ = 0x07;
  Run(seq, 801, 850);  // 5 回分
  EXPECT_EQ(seq.reinit_count(), 0u);
  EXPECT_TRUE(seq.running());
  EXPECT_EQ(seq.error_count(), 5u);
  fail_error_ = 0;
  Run(seq, 851, 870);
  EXPECT_EQ(seq.error_count(), 0u);
}

TEST_F(BNO055SequencerTest, SoftResetAfterTooManyErrors)
{
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 800);
  ASSERT_TRUE(seq.running());
  fail_error_ = 0x07;
  ops_.clear();
  Run(seq, 801, 920);  // 11 回失敗 → SOFT_RESET
  bool reset_seen = false;
  for (const auto & op : ops_) {
    if (op.kind == Op::Kind::WRITE && op.reg == 0x3F && op.data[0] == 0x20) {
      reset_seen = true;
      EXPECT_FALSE(op.expect_response);
    }
  }
  EXPECT_TRUE(reset_seen);
  EXPECT_EQ(seq.reinit_count(), 1u);
  EXPECT_FALSE(seq.running());
  EXPECT_EQ(seq.state(), State::BOOT_WAIT);
  fail_error_ = 0;
  ops_.clear();
  Run(seq, 921, 1500);  // リセットは 902 ms 付近 → 1602 ms まで boot_wait
  EXPECT_TRUE(ops_.empty());  // boot_wait_ms の間は何も出さない
  Run(seq, 1501, 1700);
  ASSERT_FALSE(ops_.empty());
  ExpectRead(ops_[0], 0x00, 1);
  EXPECT_TRUE(seq.running());
}

TEST_F(BNO055SequencerTest, WrongChipIdNeverRuns)
{
  chip_id_ = 0x55;
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 5000);
  EXPECT_FALSE(seq.running());
  EXPECT_GE(seq.reinit_count(), 1u);
  EXPECT_GT(ops_.size(), 10u);  // 固まらず操作を出し続ける
}

TEST_F(BNO055SequencerTest, SysStatusErrorReadsSysErr)
{
  sys_status_ = 1;
  sys_err_ = 3;
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 800);
  EXPECT_FALSE(seq.running());
  EXPECT_EQ(seq.sys_err(), 3);
  EXPECT_GE(CountRead(0x3A), 1);
  EXPECT_GE(CountRead(0x39), 2);  // 再試行している
  EXPECT_GE(seq.reinit_count(), 1u);  // 再試行がしきい値に達してソフトリセットした
}

TEST_F(BNO055SequencerTest, RestartGoesToSoftReset)
{
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 800);
  ASSERT_TRUE(seq.running());
  seq.Restart(801);
  ops_.clear();
  Run(seq, 802, 820);
  ASSERT_FALSE(ops_.empty());
  EXPECT_EQ(ops_[0].kind, Op::Kind::WRITE);
  EXPECT_EQ(ops_[0].reg, 0x3F);
  EXPECT_EQ(ops_[0].data[0], 0x20);
  EXPECT_FALSE(seq.running());
}

TEST_F(BNO055SequencerTest, HeadingWrapsToSignedYaw)
{
  const float heading = BNO055::ScaleEulerRad(5654);  // ≈ 2π − 0.0004
  const float yaw = Utils::WrapPi(-heading);
  EXPECT_GT(yaw, -static_cast<float>(M_PI));
  EXPECT_LT(yaw, static_cast<float>(M_PI));
  EXPECT_NEAR(yaw, 0.0f, 1e-3f);
  EXPECT_FLOAT_EQ(BNO055::ScaleGyroRad(900), 1.0f);
}

// --- 既存の換算（°単位の既定） ---

TEST(BNO055Legacy, UpdateGyroDegPerSec)
{
  BNO055 bno;
  ImuInfo imu = {};
  const uint8_t raw[6] = {16, 0, 32, 0, 0xF0, 0xFF};  // 1, 2, -1 °/s
  bno.UpdateGyro(raw, imu);
  EXPECT_NEAR(imu.gyro_x_, 0.017453f, 1e-5f);
  EXPECT_NEAR(imu.gyro_y_, 0.034907f, 1e-5f);
  EXPECT_NEAR(imu.gyro_z_, -0.017453f, 1e-5f);
}

TEST(BNO055Legacy, UpdateEulerDeg)
{
  BNO055 bno;
  ImuInfo imu = {};
  const uint8_t raw[6] = {16, 0, 0, 0, 0, 0};
  bno.UpdateEuler(raw, imu);
  EXPECT_NEAR(imu.euler_heading_, 0.017453f, 1e-5f);
}

TEST(BNO055Legacy, UpdateAccel)
{
  BNO055 bno;
  ImuInfo imu = {};
  const uint8_t raw[6] = {100, 0, 0, 0, 0, 0};
  bno.UpdateAccel(raw, imu);
  EXPECT_FLOAT_EQ(imu.acc_x_, 1.0f);
}


}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
