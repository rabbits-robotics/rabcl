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
  Run(seq, 0, 1000);  // 読み直しは 10 ms 間隔 → 11 回で約 120 ms
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

TEST_F(BNO055SequencerTest, ZeroPeriodReadsOnEveryNext)
{
  // sample_period_ms = 0: 周期は呼び出し側（ファームの tick）が持ち、RUNNING では Next のたびに読む
  BNO055Config c = DefaultConfig();
  c.sample_period_ms = 0;
  BNO055Sequencer seq(c);
  Run(seq, 0, 800);
  ASSERT_TRUE(seq.running());
  ops_.clear();
  Run(seq, 801, 850);  // 50 回呼ぶ → 50 回読む
  EXPECT_EQ(CountRead(0x14) + CountRead(0x35), 50);
}

TEST_F(BNO055SequencerTest, SysStatusInitializingWaitsWithoutError)
{
  // SYS_STATUS が 2 / 3（初期化中）なら、エラーに数えず 10 ms 待って読み直す
  sys_status_ = 3;
  BNO055Sequencer seq(DefaultConfig());
  Run(seq, 0, 760);
  EXPECT_EQ(seq.error_count(), 0u);
  EXPECT_EQ(seq.reinit_count(), 0u);
  EXPECT_EQ(CountRead(0x3A), 0);
  const int reads_before = CountRead(0x39);
  EXPECT_GE(reads_before, 2);
  EXPECT_LE(reads_before, 5);  // 約 10 ms ごと（731 〜 760 ms）
  sys_status_ = 5;
  Run(seq, 761, 800);
  EXPECT_TRUE(seq.running());
}

TEST(BNO055SequencerState, StateNames)
{
  EXPECT_STREQ(BNO055Sequencer::StateName(BNO055Sequencer::State::BOOT_WAIT), "BOOT_WAIT");
  EXPECT_STREQ(BNO055Sequencer::StateName(BNO055Sequencer::State::RUNNING), "RUNNING");
  EXPECT_STREQ(BNO055Sequencer::StateName(BNO055Sequencer::State::READ_SYS_ERR), "READ_SYS_ERR");
  EXPECT_STREQ(BNO055Sequencer::StateName(BNO055Sequencer::State::SOFT_RESET), "SOFT_RESET");
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


// --- BNO055UartDriver: UART のバイト列で偽 BNO055 とやり取り ---

// 台本どおりに答える偽 BNO055（UART）。受けた要求のバイト列を記録する
class FakeUartBno055
{
public:
  // 要求（4 + len B）を受けて応答のバイト列を返す。answer が false なら何も返さない
  std::vector<uint8_t> Respond(const uint8_t * req, size_t len)
  {
    requests.emplace_back(req, req + len);
    if (!answer) {
      return {};
    }
    std::vector<uint8_t> out;
    if (req[1] == 0x00) {  // 書き
      out = {0xEE, 0x01};
      return out;
    }
    const uint8_t reg = req[2];
    const uint8_t n = req[3];
    out.push_back(0xBB);
    out.push_back(n);
    for (uint8_t i = 0; i < n; ++i) {
      uint8_t v = 0;
      if (reg == 0x00) {
        v = chip_id;
      } else if (reg == 0x39) {
        v = 5;
      } else if (reg == 0x38) {
        v = 0;
      } else if (reg == 0x35) {
        v = 0xC0;
      } else if (reg == 0x14) {
        v = data[i];
      }
      out.push_back(v);
    }
    return out;
  }

  std::vector<std::vector<uint8_t>> requests;
  bool answer = true;
  uint8_t chip_id = 0xA0;
  uint8_t data[12] = {};
};

class BNO055UartDriverTest : public ::testing::Test
{
protected:
  // Tick → 偽 BNO055 → Feed を回す。応答は fake が返した分を一度に Feed
  int Run(BNO055UartDriver & driver, uint32_t from_ms, uint32_t to_ms)
  {
    int samples = 0;
    for (uint32_t t = from_ms; t <= to_ms; ++t) {
      uint8_t tx[16];
      const size_t n = driver.Tick(t, tx, sizeof(tx));
      if (n == 0) {
        continue;
      }
      const std::vector<uint8_t> reply = fake_.Respond(tx, n);
      if (!reply.empty() && driver.Feed(reply.data(), reply.size(), t, sample_)) {
        ++samples;
      }
    }
    return samples;
  }

  static std::vector<uint8_t> Bytes(std::initializer_list<uint8_t> b)
  {
    return std::vector<uint8_t>(b);
  }

  FakeUartBno055 fake_;
  BNO055Sample sample_ = {};
};

TEST_F(BNO055UartDriverTest, BootSequenceBytes)
{
  BNO055UartDriver driver(BNO055Config{});
  Run(driver, 0, 800);
  ASSERT_GE(fake_.requests.size(), 8u);
  EXPECT_EQ(fake_.requests[0], Bytes({0xAA, 0x01, 0x00, 0x01}));
  EXPECT_EQ(fake_.requests[1], Bytes({0xAA, 0x00, 0x3D, 0x01, 0x00}));
  EXPECT_EQ(fake_.requests[2], Bytes({0xAA, 0x00, 0x07, 0x01, 0x00}));
  EXPECT_EQ(fake_.requests[3], Bytes({0xAA, 0x00, 0x3B, 0x01, 0x06}));
  EXPECT_EQ(fake_.requests[4], Bytes({0xAA, 0x00, 0x41, 0x01, 0x24}));
  EXPECT_EQ(fake_.requests[5], Bytes({0xAA, 0x00, 0x42, 0x01, 0x00}));
  EXPECT_EQ(fake_.requests[6], Bytes({0xAA, 0x00, 0x3D, 0x01, 0x08}));
  EXPECT_EQ(fake_.requests[7], Bytes({0xAA, 0x01, 0x39, 0x01}));
  EXPECT_TRUE(driver.sequencer().running());
}

TEST_F(BNO055UartDriverTest, ExpectedResponseLen)
{
  BNO055UartDriver driver(BNO055Config{});
  EXPECT_EQ(driver.expected_response_len(), 0u);
  uint8_t tx[16];
  ASSERT_EQ(driver.Tick(700, tx, sizeof(tx)), 4u);  // CHIP_ID 読み
  EXPECT_EQ(driver.expected_response_len(), 3u);
  const uint8_t id[3] = {0xBB, 0x01, 0xA0};
  driver.Feed(id, 3, 700, sample_);
  EXPECT_EQ(driver.expected_response_len(), 0u);
  ASSERT_EQ(driver.Tick(701, tx, sizeof(tx)), 5u);  // OPR_MODE 書き
  EXPECT_EQ(driver.expected_response_len(), 2u);
  const uint8_t ack[2] = {0xEE, 0x01};
  driver.Feed(ack, 2, 701, sample_);
  Run(driver, 702, 800);
  ASSERT_TRUE(driver.sequencer().running());
  Run(driver, 801, 810);  // RUNNING は 731 ms から 10 ms ごと → 801 で読み、次は 811
  ASSERT_EQ(driver.Tick(811, tx, sizeof(tx)), 4u);  // データ読み
  EXPECT_EQ(driver.expected_response_len(), 14u);
}

TEST_F(BNO055UartDriverTest, SampleFromSplitBytes)
{
  fake_.data[4] = 900 & 0xFF;
  fake_.data[5] = 900 >> 8;
  BNO055UartDriver driver(BNO055Config{});
  Run(driver, 0, 800);
  ASSERT_TRUE(driver.sequencer().running());
  uint8_t tx[16];
  Run(driver, 801, 810);
  ASSERT_EQ(driver.Tick(811, tx, sizeof(tx)), 4u);
  const std::vector<uint8_t> reply = fake_.Respond(tx, 4);
  ASSERT_EQ(reply.size(), 14u);
  EXPECT_FALSE(driver.Feed(reply.data(), 3, 811, sample_));
  EXPECT_TRUE(driver.Feed(reply.data() + 3, 11, 812, sample_));
  EXPECT_FLOAT_EQ(sample_.gyro[2], 1.0f);
  EXPECT_EQ(driver.last_request_ms(), 811u);
}

TEST_F(BNO055UartDriverTest, LateResponseAfterTimeoutIsIgnored)
{
  BNO055UartDriver driver(BNO055Config{});
  Run(driver, 0, 800);
  ASSERT_TRUE(driver.sequencer().running());
  uint8_t tx[16];
  Run(driver, 801, 810);
  ASSERT_EQ(driver.Tick(811, tx, sizeof(tx)), 4u);  // データ読み。応答しない
  const std::vector<uint8_t> late = fake_.Respond(tx, 4);
  // 20 ms 後に再送される
  size_t resent = 0;
  for (uint32_t t = 812; t <= 840; ++t) {
    resent = driver.Tick(t, tx, sizeof(tx));
    if (resent > 0) {
      break;
    }
  }
  ASSERT_EQ(resent, 4u);
  EXPECT_EQ(driver.sequencer().error_count(), 1u);
  // 再送の直後に「前の応答」が届いても、形が合うので正当な応答として受ける（内容は同じ）。
  // 待っていないときに届いたバイトは捨てられる: 応答を 2 回続けて入れると 2 回目は無視
  EXPECT_TRUE(driver.Feed(late.data(), late.size(), 841, sample_));
  EXPECT_FALSE(driver.Feed(late.data(), late.size(), 842, sample_));
  EXPECT_FALSE(driver.sequencer().waiting());
  EXPECT_EQ(driver.sequencer().error_count(), 0u);
}

TEST_F(BNO055UartDriverTest, GarbageWhileIdleIsIgnored)
{
  BNO055UartDriver driver(BNO055Config{});
  const uint8_t garbage[6] = {0xFF, 0xFF, 0xBB, 0x05, 0x01, 0x02};
  EXPECT_FALSE(driver.Feed(garbage, sizeof(garbage), 100, sample_));
  EXPECT_EQ(driver.sequencer().state(), BNO055Sequencer::State::BOOT_WAIT);
  Run(driver, 101, 800);
  EXPECT_TRUE(driver.sequencer().running());
  EXPECT_EQ(driver.sequencer().error_count(), 0u);
}

TEST_F(BNO055UartDriverTest, SoftResetIsSentWithoutWaiting)
{
  BNO055UartDriver driver(BNO055Config{});
  fake_.chip_id = 0x55;  // CHIP_ID が合わない → 連続エラー → ソフトリセット
  Run(driver, 0, 1000);
  bool reset_seen = false;
  for (const auto & req : fake_.requests) {
    if (req == Bytes({0xAA, 0x00, 0x3F, 0x01, 0x20})) {
      reset_seen = true;
    }
  }
  EXPECT_TRUE(reset_seen);
  EXPECT_GE(driver.sequencer().reinit_count(), 1u);
  EXPECT_EQ(driver.expected_response_len(), 0u);
}

TEST_F(BNO055UartDriverTest, LastRequestMs)
{
  BNO055UartDriver driver(BNO055Config{});
  uint8_t tx[16];
  EXPECT_EQ(driver.Tick(500, tx, sizeof(tx)), 0u);
  ASSERT_EQ(driver.Tick(1234, tx, sizeof(tx)), 4u);
  EXPECT_EQ(driver.last_request_ms(), 1234u);
}

// --- BNO055I2cDriver: 抽象レジスタバスの偽実装 ---

class FakeRegisterBus : public BNO055RegisterBus
{
public:
  FakeRegisterBus()
  {
    regs[0x00] = 0xA0;
    regs[0x39] = 5;
    regs[0x35] = 0xC0;
  }
  bool Read(uint8_t reg, uint8_t * data, uint8_t len) override
  {
    reads.emplace_back(reg, len);
    if (fail_next > 0) {
      --fail_next;
      return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
      data[i] = regs[static_cast<uint8_t>(reg + i)];
    }
    return true;
  }
  bool Write(uint8_t reg, const uint8_t * data, uint8_t len) override
  {
    writes.emplace_back(reg, std::vector<uint8_t>(data, data + len));
    if (fail_next > 0) {
      --fail_next;
      return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
      regs[static_cast<uint8_t>(reg + i)] = data[i];
    }
    return true;
  }

  uint8_t regs[256] = {};
  std::vector<std::pair<uint8_t, uint8_t>> reads;
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> writes;
  int fail_next = 0;
};

class BNO055I2cDriverTest : public ::testing::Test
{
protected:
  int Run(BNO055I2cDriver & driver, uint32_t from_ms, uint32_t to_ms)
  {
    int samples = 0;
    for (uint32_t t = from_ms; t <= to_ms; ++t) {
      if (driver.Tick(t, sample_)) {
        ++samples;
      }
    }
    return samples;
  }

  FakeRegisterBus bus_;
  BNO055Sample sample_ = {};
};

TEST_F(BNO055I2cDriverTest, BootSequenceOnBus)
{
  BNO055I2cDriver driver(BNO055Config{}, bus_);
  Run(driver, 0, 800);
  ASSERT_GE(bus_.writes.size(), 6u);
  EXPECT_EQ(bus_.writes[0].first, 0x3D);
  EXPECT_EQ(bus_.writes[0].second[0], 0x00);
  EXPECT_EQ(bus_.writes[1].first, 0x07);
  EXPECT_EQ(bus_.writes[2].first, 0x3B);
  EXPECT_EQ(bus_.writes[3].first, 0x41);
  EXPECT_EQ(bus_.writes[4].first, 0x42);
  EXPECT_EQ(bus_.writes[5].first, 0x3D);
  EXPECT_EQ(bus_.writes[5].second[0], 0x08);
  ASSERT_GE(bus_.reads.size(), 2u);
  EXPECT_EQ(bus_.reads[0].first, 0x00);
  EXPECT_EQ(bus_.reads[1].first, 0x39);
  EXPECT_TRUE(driver.sequencer().running());
}

TEST_F(BNO055I2cDriverTest, SampleFromBus)
{
  bus_.regs[0x18] = 900 & 0xFF;  // gyro z
  bus_.regs[0x19] = 900 >> 8;
  bus_.regs[0x1A] = 2827 & 0xFF;  // heading
  bus_.regs[0x1B] = 2827 >> 8;
  BNO055I2cDriver driver(BNO055Config{}, bus_);
  Run(driver, 0, 800);
  ASSERT_TRUE(driver.sequencer().running());
  const int samples = Run(driver, 801, 1000);
  EXPECT_EQ(samples, 20);
  EXPECT_FLOAT_EQ(sample_.gyro[2], 1.0f);
  EXPECT_NEAR(sample_.euler[0], static_cast<float>(M_PI), 1e-3f);
}

TEST_F(BNO055I2cDriverTest, BusFailureCountsAndResets)
{
  BNO055I2cDriver driver(BNO055Config{}, bus_);
  Run(driver, 0, 800);
  ASSERT_TRUE(driver.sequencer().running());
  bus_.fail_next = 11;
  Run(driver, 801, 1000);
  EXPECT_EQ(driver.sequencer().reinit_count(), 1u);
  bool reset_seen = false;
  for (const auto & w : bus_.writes) {
    if (w.first == 0x3F && w.second[0] == 0x20) {
      reset_seen = true;
    }
  }
  EXPECT_TRUE(reset_seen);
  EXPECT_FALSE(driver.sequencer().running());
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
