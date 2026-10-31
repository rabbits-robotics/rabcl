#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <vector>

#include "rabcl/component/st3215.hpp"

namespace rabcl
{

using P = St3215Protocol;

// 送信を記録し、用意した応答（エコーは含めない）を返す偽のバス
class FakePort : public St3215Port
{
public:
  std::vector<std::vector<uint8_t>> sent;
  std::vector<uint8_t> reply;
  std::vector<std::vector<uint8_t>> replies;  // 呼び出しごとの応答（空 = 応答なし）。尽きたら reply
  int result = 0;  // 0 以外なら Transfer の戻り値をこれにする

  int Transfer(const uint8_t * tx, size_t tx_len, uint8_t * rx, size_t rx_len, uint32_t) override
  {
    const size_t call = sent.size();
    sent.emplace_back(tx, tx + tx_len);
    const std::vector<uint8_t> & r = call < replies.size() ? replies[call] : reply;
    const size_t n = std::min(rx_len, r.size());
    std::memcpy(rx, r.data(), n);
    return result != 0 ? result : static_cast<int>(n);
  }
};

const std::vector<uint8_t> ACK = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};

TEST(St3215Protocol, BuildPing)
{
  uint8_t b[8];
  ASSERT_EQ(6u, P::BuildPing(1, b));
  const uint8_t e[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  EXPECT_EQ(0, std::memcmp(b, e, 6));
}

TEST(St3215Protocol, BuildRead)
{
  uint8_t b[8];
  ASSERT_EQ(8u, P::BuildRead(1, 56, 2, b));
  const uint8_t e[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};
  EXPECT_EQ(0, std::memcmp(b, e, 8));
}

TEST(St3215Protocol, BuildWrite)
{
  uint8_t b[16];
  const uint8_t d[] = {0xAB, 0x00};
  ASSERT_EQ(9u, P::BuildWrite(1, 42, d, 2, b));
  const uint8_t e[] = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0xAB, 0x00, 0x21};
  EXPECT_EQ(0, std::memcmp(b, e, 9));
}

TEST(St3215Protocol, SignedEncoding)
{
  EXPECT_EQ(0x8064, P::EncodeSigned(-100));
  EXPECT_EQ(171, P::EncodeSigned(171));
  EXPECT_EQ(-100, P::DecodeSigned(0x8064));
  EXPECT_EQ(1234, P::DecodeSigned(1234));
  EXPECT_EQ(-100, P::DecodeLoad(0x0464));
  EXPECT_EQ(100, P::DecodeLoad(0x0064));
}

TEST(St3215Protocol, LittleEndian)
{
  uint8_t b[2];
  P::PutU16(0x03E8, b);
  EXPECT_EQ(0xE8, b[0]);
  EXPECT_EQ(0x03, b[1]);
  EXPECT_EQ(0x03E8, P::GetU16(b));
}

TEST(St3215Protocol, ResponseLength)
{
  EXPECT_EQ(6u, P::ResponseLength(0));
  EXPECT_EQ(8u, P::ResponseLength(2));
}

TEST(St3215Protocol, ParseAck)
{
  P::Response o;
  size_t c = 0;
  ASSERT_TRUE(P::ParseResponse(ACK.data(), ACK.size(), o, c));
  EXPECT_EQ(1, o.id);
  EXPECT_EQ(0, o.error);
  EXPECT_EQ(0, o.len);
  EXPECT_EQ(6u, c);
}

TEST(St3215Protocol, ParseReadData)
{
  const uint8_t r[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0xD2, 0x04, 0x24};
  P::Response o;
  size_t c = 0;
  ASSERT_TRUE(P::ParseResponse(r, 8, o, c));
  EXPECT_EQ(2, o.len);
  EXPECT_EQ(1234, P::GetU16(o.data));
  EXPECT_EQ(8u, c);
}

TEST(St3215Protocol, ParseResponseSkipsLeadingGarbage)
{
  const uint8_t r[] = {0x12, 0x34, 0xFF, 0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  P::Response o;
  size_t c = 0;
  ASSERT_TRUE(P::ParseResponse(r, 9, o, c));
  EXPECT_EQ(1, o.id);
  EXPECT_EQ(9u, c);
}

TEST(St3215Protocol, ParseResponseRejectsBadChecksum)
{
  const uint8_t r[] = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFD};
  P::Response o;
  size_t c = 0;
  EXPECT_FALSE(P::ParseResponse(r, 6, o, c));
}

TEST(St3215Protocol, ParseResponseRejectsShort)
{
  const uint8_t r[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0xD2};
  P::Response o;
  size_t c = 0;
  EXPECT_FALSE(P::ParseResponse(r, 6, o, c));
}

TEST(St3215, MoveWritesAccPositionTimeSpeed)
{
  FakePort port;
  port.reply = ACK;
  St3215 s(port, 1);
  ASSERT_TRUE(s.Move(171, 1000, 50));
  const uint8_t e[] = {0xFF, 0xFF, 0x01, 0x0A, 0x03, 0x29, 0x32, 0xAB, 0x00, 0x00, 0x00, 0xE8, 0x03,
    0x00};
  ASSERT_EQ(1u, port.sent.size());
  EXPECT_EQ(std::vector<uint8_t>(e, e + 14), port.sent[0]);
}

TEST(St3215, MoveNegativeStepUsesSignBit)
{
  FakePort port;
  port.reply = ACK;
  St3215 s(port, 1);
  ASSERT_TRUE(s.Move(-171, 1000, 50));
  EXPECT_EQ(0xAB, port.sent[0][7]);
  EXPECT_EQ(0x80, port.sent[0][8]);
}

TEST(St3215, ReadStatusDecodesFields)
{
  FakePort port;
  St3215 s(port, 1);
  // 56..70 の 15 バイト: pos 1234, speed -50, load -100 (bit10), volt 74, temp 31, [64,65]=0,
  // moving 1, [67,68]=0, current -7
  const uint8_t d[15] = {0xD2, 0x04, 0x32, 0x80, 0x64, 0x04, 74, 31, 0, 0, 1, 0, 0, 0x07, 0x80};
  uint8_t r[21] = {0xFF, 0xFF, 0x01, 0x11, 0x00};
  std::memcpy(r + 5, d, 15);
  uint8_t sum = 0x01 + 0x11 + 0x00;
  for (uint8_t v : d) {
    sum = static_cast<uint8_t>(sum + v);
  }
  r[20] = static_cast<uint8_t>(~sum);
  port.reply.assign(r, r + 21);
  St3215Status st;
  ASSERT_TRUE(s.ReadStatus(st));
  EXPECT_EQ(1234, st.position);
  EXPECT_EQ(-50, st.speed);
  EXPECT_EQ(-100, st.load);
  EXPECT_EQ(74, st.voltage);
  EXPECT_EQ(31, st.temperature);
  EXPECT_TRUE(st.moving);
  EXPECT_EQ(-7, st.current);
  EXPECT_EQ(0x38, port.sent[0][5]);
  EXPECT_EQ(15, port.sent[0][6]);
}

const std::vector<uint8_t> READ_MODE_3 = {0xFF, 0xFF, 0x01, 0x03, 0x00, 0x03, 0xF8};
const std::vector<uint8_t> READ_LOCK_1 = {0xFF, 0xFF, 0x01, 0x03, 0x00, 0x01, 0xFA};

// LOCK 0 を書く → MODE を書く → MODE を読み戻す → LOCK 1 を書く → LOCK を読み戻す
TEST(St3215, SetModeUnlocksWritesLocksAndReadsBack)
{
  FakePort port;
  port.replies = {ACK, ACK, READ_MODE_3, ACK, READ_LOCK_1};
  St3215 s(port, 1);
  ASSERT_TRUE(s.SetMode(P::Mode::STEP));
  ASSERT_EQ(5u, port.sent.size());
  EXPECT_EQ(P::INST_WRITE, port.sent[0][4]);
  EXPECT_EQ(55, port.sent[0][5]);
  EXPECT_EQ(0, port.sent[0][6]);
  EXPECT_EQ(P::INST_WRITE, port.sent[1][4]);
  EXPECT_EQ(33, port.sent[1][5]);
  EXPECT_EQ(3, port.sent[1][6]);
  EXPECT_EQ(P::INST_READ, port.sent[2][4]);
  EXPECT_EQ(33, port.sent[2][5]);
  EXPECT_EQ(P::INST_WRITE, port.sent[3][4]);
  EXPECT_EQ(55, port.sent[3][5]);
  EXPECT_EQ(1, port.sent[3][6]);
  EXPECT_EQ(P::INST_READ, port.sent[4][4]);
  EXPECT_EQ(55, port.sent[4][5]);
}

// 実測: EEPROM（MODE）を書いた直後はサーボの応答が遅れ、書き込みと次の LOCK 書き込みの応答が揃わないことがある。
// 書き込みの応答ではなく読み戻しで判定するので成功になる
TEST(St3215, SetModeSucceedsWhenEepromWriteAcksAreLate)
{
  FakePort port;
  port.replies = {ACK, {}, READ_MODE_3, {}, READ_LOCK_1};
  St3215 s(port, 1);
  EXPECT_TRUE(s.SetMode(P::Mode::STEP));
  EXPECT_EQ(2u, s.timeouts());
}

// 読み戻しが 1 回目で遅れた応答（書き込みの ACK）に化けても、1 回だけ読み直して判定する
TEST(St3215, SetModeRetriesReadbackOnce)
{
  FakePort port;
  port.replies = {ACK, {}, ACK, READ_MODE_3, ACK, READ_LOCK_1};
  St3215 s(port, 1);
  EXPECT_TRUE(s.SetMode(P::Mode::STEP));
  EXPECT_EQ(6u, port.sent.size());
}

TEST(St3215, SetModeFailsWhenReadbackDiffers)
{
  FakePort port;
  const std::vector<uint8_t> read_mode_0 = {0xFF, 0xFF, 0x01, 0x03, 0x00, 0x00, 0xFB};
  port.replies = {ACK, ACK, read_mode_0, read_mode_0, ACK, READ_LOCK_1};
  St3215 s(port, 1);
  EXPECT_FALSE(s.SetMode(P::Mode::STEP));
  ASSERT_GE(port.sent.size(), 5u);
  EXPECT_EQ(55, port.sent[4][5]);  // 失敗しても LOCK 1 は書く
  EXPECT_EQ(1, port.sent[4][6]);
}

TEST(St3215, SetModeFailsWhenUnlockFails)
{
  FakePort port;
  port.replies = {{}};
  St3215 s(port, 1);
  EXPECT_FALSE(s.SetMode(P::Mode::STEP));
  EXPECT_EQ(1u, port.sent.size());
}

TEST(St3215, NoResponseCountsTimeout)
{
  FakePort port;
  St3215 s(port, 1);
  EXPECT_FALSE(s.Ping());
  EXPECT_EQ(1u, s.timeouts());
}

TEST(St3215, ErrorByteIsKept)
{
  FakePort port;
  port.reply = {0xFF, 0xFF, 0x01, 0x02, 0x20, 0xDC};
  St3215 s(port, 1);
  EXPECT_TRUE(s.Ping());
  EXPECT_EQ(0x20, s.last_error());
}

}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
