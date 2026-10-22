#include <cstring>
#include <vector>
#include <gtest/gtest.h>
#include "rabcl/protocol/parser.hpp"
#include "rabcl/protocol/protocol.hpp"

namespace rabcl::protocol
{

namespace
{

void PutFloat(uint8_t * buf, float v)
{
  std::memcpy(buf, &v, sizeof(v));
}

// 受信したフレームを溜める
struct Received
{
  struct Frame
  {
    Type type;
    std::vector<uint8_t> payload;
  };
  std::vector<Frame> frames;

  static void Handler(Type type, const uint8_t * payload, uint8_t len, void * ctx)
  {
    auto * self = static_cast<Received *>(ctx);
    self->frames.push_back({type, std::vector<uint8_t>(payload, payload + len)});
  }
};

std::vector<uint8_t> ChassisVelFrame(float vx, float vy, float wz)
{
  std::vector<uint8_t> buf(MAX_FRAME);
  const size_t n = EncodeChassisVel({vx, vy, wz}, buf.data(), buf.size());
  buf.resize(n);
  return buf;
}

std::vector<uint8_t> ModeFrame()
{
  const ModeCmd modes[2] = {{0, 1}, {3, 2}};
  std::vector<uint8_t> buf(MAX_FRAME);
  const size_t n = EncodeMode(modes, 2, buf.data(), buf.size());
  buf.resize(n);
  return buf;
}

}  // namespace

// --- CRC / Encode / Decode ---

TEST(Protocol, Crc8KnownValue)
{
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(Crc8(check, sizeof(check)), 0xF4);
}

TEST(Protocol, EncodeChassisVelFrameBytes)
{
  uint8_t buf[MAX_FRAME];
  const size_t n = EncodeChassisVel({1.0f, -2.0f, 0.5f}, buf, sizeof(buf));
  ASSERT_EQ(n, 17u);
  EXPECT_EQ(buf[0], HEADER_0);
  EXPECT_EQ(buf[1], HEADER_1);
  EXPECT_EQ(buf[2], static_cast<uint8_t>(Type::CHASSIS_VEL));
  EXPECT_EQ(buf[3], 12);
  uint8_t expected[12];
  PutFloat(&expected[0], 1.0f);
  PutFloat(&expected[4], -2.0f);
  PutFloat(&expected[8], 0.5f);
  EXPECT_EQ(std::memcmp(&buf[4], expected, 12), 0);
  EXPECT_EQ(buf[16], Crc8(&buf[2], 14));
}

TEST(Protocol, InfoRoundTrip)
{
  Info in = {};
  in.protocol_version = 1;
  in.joint_count = 11;
  std::strncpy(in.firmware_id, "gamma", sizeof(in.firmware_id));
  uint8_t buf[MAX_FRAME];
  const size_t n = EncodeInfo(in, buf, sizeof(buf));
  ASSERT_EQ(n, 5u + 10u);
  Info out = {};
  ASSERT_TRUE(DecodeInfo(&buf[4], buf[3], out));
  EXPECT_EQ(out.protocol_version, 1);
  EXPECT_EQ(out.joint_count, 11);
  EXPECT_EQ(std::memcmp(out.firmware_id, "gamma\0\0\0", 8), 0);
}

TEST(Protocol, JointStateRoundTrip)
{
  const JointState in[3] = {
    {0, 1.5f, -2.5f, 3.5f, 40.0f, 0x00},
    {7, 0.468f, -0.007f, 0.002f, 28.0f, 0x01},
    {10, -12.5f, 30.0f, -10.0f, 100.0f, 0x0A},
  };
  uint8_t buf[MAX_FRAME];
  const size_t n = EncodeJointState(in, 3, buf, sizeof(buf));
  ASSERT_EQ(n, 5u + 1u + 18u * 3u);
  JointState out[3] = {};
  uint8_t count = 0;
  ASSERT_TRUE(DecodeJointState(&buf[4], buf[3], out, 3, count));
  ASSERT_EQ(count, 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(out[i].id, in[i].id);
    EXPECT_FLOAT_EQ(out[i].position, in[i].position);
    EXPECT_FLOAT_EQ(out[i].velocity, in[i].velocity);
    EXPECT_FLOAT_EQ(out[i].effort, in[i].effort);
    EXPECT_FLOAT_EQ(out[i].temperature, in[i].temperature);
    EXPECT_EQ(out[i].error, in[i].error);
  }
}

TEST(Protocol, JointStateDecodeRejectsWrongLen)
{
  const JointState in[2] = {{0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0}};
  uint8_t buf[MAX_FRAME];
  ASSERT_GT(EncodeJointState(in, 2, buf, sizeof(buf)), 0u);
  JointState out[2] = {};
  uint8_t count = 0;
  EXPECT_FALSE(DecodeJointState(&buf[4], static_cast<uint8_t>(buf[3] + 1), out, 2, count));
}

TEST(Protocol, EncodeJointStateRejectsMoreThan14)
{
  JointState in[15] = {};
  uint8_t buf[MAX_FRAME];
  EXPECT_EQ(EncodeJointState(in, 15, buf, sizeof(buf)), 0u);
  EXPECT_GT(EncodeJointState(in, 14, buf, sizeof(buf)), 0u);
}

TEST(Protocol, EncodeReturnsZeroWhenBufferTooSmall)
{
  uint8_t buf[16];  // ChassisVel は 17 B
  EXPECT_EQ(EncodeChassisVel({0, 0, 0}, buf, sizeof(buf)), 0u);
}

TEST(Protocol, ImuRoundTrip)
{
  Imu in = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  uint8_t buf[MAX_FRAME];
  ASSERT_EQ(EncodeImu(in, buf, sizeof(buf)), 5u + 36u);
  Imu out = {};
  ASSERT_TRUE(DecodeImu(&buf[4], buf[3], out));
  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(out.acc[i], in.acc[i]);
    EXPECT_FLOAT_EQ(out.gyro[i], in.gyro[i]);
    EXPECT_FLOAT_EQ(out.euler[i], in.euler[i]);
  }
  EXPECT_FALSE(DecodeImu(&buf[4], 35, out));
}

TEST(Protocol, SystemRoundTrip)
{
  System in = {24.1f, 0x80000001u, 123456u};
  uint8_t buf[MAX_FRAME];
  ASSERT_EQ(EncodeSystem(in, buf, sizeof(buf)), 5u + 12u);
  System out = {};
  ASSERT_TRUE(DecodeSystem(&buf[4], buf[3], out));
  EXPECT_FLOAT_EQ(out.bus_voltage, 24.1f);
  EXPECT_EQ(out.error_flags, 0x80000001u);
  EXPECT_EQ(out.uptime_ms, 123456u);
}

TEST(Protocol, ChassisVelRoundTrip)
{
  uint8_t buf[MAX_FRAME];
  ASSERT_GT(EncodeChassisVel({0.1f, 0.2f, -0.3f}, buf, sizeof(buf)), 0u);
  ChassisVel out = {};
  ASSERT_TRUE(DecodeChassisVel(&buf[4], buf[3], out));
  EXPECT_FLOAT_EQ(out.vx, 0.1f);
  EXPECT_FLOAT_EQ(out.vy, 0.2f);
  EXPECT_FLOAT_EQ(out.wz, -0.3f);
}

TEST(Protocol, JointCmdRoundTrip)
{
  JointCmd in[12];
  for (int i = 0; i < 12; ++i) {
    in[i] = {static_cast<uint8_t>(i), 1.0f * i, 2.0f * i, 3.0f * i, 4.0f * i, 5.0f * i};
  }
  uint8_t buf[MAX_FRAME];
  ASSERT_EQ(EncodeJointCmd(in, 12, buf, sizeof(buf)), 5u + 1u + 21u * 12u);
  JointCmd out[12] = {};
  uint8_t count = 0;
  ASSERT_TRUE(DecodeJointCmd(&buf[4], buf[3], out, 12, count));
  ASSERT_EQ(count, 12);
  EXPECT_EQ(out[11].id, 11);
  EXPECT_FLOAT_EQ(out[11].position, 11.0f);
  EXPECT_FLOAT_EQ(out[11].velocity, 22.0f);
  EXPECT_FLOAT_EQ(out[11].effort, 33.0f);
  EXPECT_FLOAT_EQ(out[11].kp, 44.0f);
  EXPECT_FLOAT_EQ(out[11].kd, 55.0f);
}

TEST(Protocol, EncodeJointCmdRejectsMoreThan12)
{
  JointCmd in[13] = {};
  uint8_t buf[MAX_FRAME];
  EXPECT_EQ(EncodeJointCmd(in, 13, buf, sizeof(buf)), 0u);
}

TEST(Protocol, ModeRoundTrip)
{
  const ModeCmd in[2] = {{0, 1}, {3, 2}};
  uint8_t buf[MAX_FRAME];
  ASSERT_EQ(EncodeMode(in, 2, buf, sizeof(buf)), 5u + 1u + 2u * 2u);
  ModeCmd out[4] = {};
  uint8_t count = 0;
  ASSERT_TRUE(DecodeMode(&buf[4], buf[3], out, 4, count));
  ASSERT_EQ(count, 2);
  EXPECT_EQ(out[0].id, 0);
  EXPECT_EQ(out[0].value, 1);
  EXPECT_EQ(out[1].id, 3);
  EXPECT_EQ(out[1].value, 2);
}

TEST(Protocol, DecodeRejectsCountOverMax)
{
  const JointState in[3] = {};
  uint8_t buf[MAX_FRAME];
  ASSERT_GT(EncodeJointState(in, 3, buf, sizeof(buf)), 0u);
  JointState out[2] = {};
  uint8_t count = 0;
  EXPECT_FALSE(DecodeJointState(&buf[4], buf[3], out, 2, count));
}

// --- Parser ---

TEST(Parser, SingleFrameInOneFeed)
{
  Parser parser;
  Received rx;
  const auto frame = ChassisVelFrame(1.0f, 2.0f, 3.0f);
  parser.Feed(frame.data(), frame.size(), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 1u);
  EXPECT_EQ(rx.frames[0].type, Type::CHASSIS_VEL);
  ASSERT_EQ(rx.frames[0].payload.size(), 12u);
  EXPECT_EQ(std::memcmp(rx.frames[0].payload.data(), &frame[4], 12), 0);
  EXPECT_EQ(parser.crc_errors(), 0u);
}

TEST(Parser, TwoFramesInOneFeed)
{
  Parser parser;
  Received rx;
  auto bytes = ChassisVelFrame(1.0f, 2.0f, 3.0f);
  const auto second = ModeFrame();
  bytes.insert(bytes.end(), second.begin(), second.end());
  parser.Feed(bytes.data(), bytes.size(), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 2u);
  EXPECT_EQ(rx.frames[0].type, Type::CHASSIS_VEL);
  EXPECT_EQ(rx.frames[1].type, Type::MODE);
}

TEST(Parser, ParserHandlesEverySplitPoint)
{
  auto bytes = ChassisVelFrame(1.0f, 2.0f, 3.0f);
  const auto second = ModeFrame();
  bytes.insert(bytes.end(), second.begin(), second.end());
  for (size_t split = 1; split < bytes.size(); ++split) {
    Parser parser;
    Received rx;
    parser.Feed(bytes.data(), split, Received::Handler, &rx);
    parser.Feed(bytes.data() + split, bytes.size() - split, Received::Handler, &rx);
    ASSERT_EQ(rx.frames.size(), 2u) << "split at " << split;
    EXPECT_EQ(rx.frames[0].type, Type::CHASSIS_VEL) << "split at " << split;
    EXPECT_EQ(rx.frames[1].type, Type::MODE) << "split at " << split;
    EXPECT_EQ(parser.crc_errors(), 0u) << "split at " << split;
  }
}

TEST(Parser, GarbageThenFrameResyncs)
{
  Parser parser;
  Received rx;
  std::vector<uint8_t> bytes = {0x00, 0x11, HEADER_0, 0x22};
  const auto frame = ChassisVelFrame(1.0f, 2.0f, 3.0f);
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  parser.Feed(bytes.data(), bytes.size(), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 1u);
  EXPECT_EQ(rx.frames[0].type, Type::CHASSIS_VEL);
  EXPECT_GE(parser.resyncs(), 1u);
}

TEST(Parser, CrcErrorIsCountedAndNextFrameIsReceived)
{
  Parser parser;
  Received rx;
  auto bad = ChassisVelFrame(1.0f, 2.0f, 3.0f);
  bad.back() ^= 0xFF;
  const auto good = ModeFrame();
  bad.insert(bad.end(), good.begin(), good.end());
  parser.Feed(bad.data(), bad.size(), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 1u);
  EXPECT_EQ(rx.frames[0].type, Type::MODE);
  EXPECT_EQ(parser.crc_errors(), 1u);
}

TEST(Parser, ParserIgnoresHeaderBytesInsidePayload)
{
  // vx のビット列を 0x3F805AA5 にすると LE で A5 5A 80 3F が payload に現れる
  float vx;
  const uint32_t bits = 0x3F805AA5u;
  std::memcpy(&vx, &bits, sizeof(vx));
  Parser parser;
  Received rx;
  auto bytes = ChassisVelFrame(vx, 0.0f, 0.0f);
  ASSERT_EQ(bytes[4], HEADER_0);
  ASSERT_EQ(bytes[5], HEADER_1);
  const auto second = ModeFrame();
  bytes.insert(bytes.end(), second.begin(), second.end());
  parser.Feed(bytes.data(), bytes.size(), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 2u);
  EXPECT_EQ(rx.frames[0].type, Type::CHASSIS_VEL);
  EXPECT_EQ(rx.frames[1].type, Type::MODE);
  EXPECT_EQ(parser.crc_errors(), 0u);
}

TEST(Parser, TruncatedFrameThenNewFrame)
{
  Parser parser;
  Received rx;
  auto truncated = ChassisVelFrame(1.0f, 2.0f, 3.0f);
  truncated.resize(9);  // 途中で切れる
  const auto good = ModeFrame();
  truncated.insert(truncated.end(), good.begin(), good.end());
  parser.Feed(truncated.data(), truncated.size(), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 1u);
  EXPECT_EQ(rx.frames[0].type, Type::MODE);
}

TEST(Parser, ZeroLengthPayloadFrame)
{
  Parser parser;
  Received rx;
  uint8_t frame[5] = {HEADER_0, HEADER_1, 0x7F, 0x00, 0x00};
  frame[4] = Crc8(&frame[2], 2);
  parser.Feed(frame, sizeof(frame), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 1u);
  EXPECT_EQ(static_cast<uint8_t>(rx.frames[0].type), 0x7F);
  EXPECT_TRUE(rx.frames[0].payload.empty());
}

TEST(Parser, NestedCrcFailuresDoNotLoseFollowingFrame)
{
  // 外側のフレーム（len 255）の body に、CRC の壊れた 5 B のフレームを 51 個詰める。
  // 再走査が 1 B ずつずれて 50 段以上入れ子になる最悪形。最後の正しいフレームだけが受かる
  std::vector<uint8_t> bytes = {HEADER_0, HEADER_1, 0x01, 0xFF};
  uint8_t bad[5] = {HEADER_0, HEADER_1, 0x01, 0x00, 0x00};
  bad[4] = static_cast<uint8_t>(Crc8(&bad[2], 2) ^ 0xFF);
  for (int i = 0; i < 51; ++i) {
    bytes.insert(bytes.end(), bad, bad + 5);
  }
  bytes.push_back(0x00);  // 外側の body + crc = 256 B に揃える
  const auto good = ModeFrame();
  bytes.insert(bytes.end(), good.begin(), good.end());

  Parser parser;
  Received rx;
  parser.Feed(bytes.data(), bytes.size(), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 1u);
  EXPECT_EQ(rx.frames[0].type, Type::MODE);
  EXPECT_GE(parser.crc_errors(), 50u);
}

TEST(Parser, MaxLengthFrameRoundTrip)
{
  uint8_t frame[MAX_FRAME];
  frame[0] = HEADER_0;
  frame[1] = HEADER_1;
  frame[2] = 0x02;
  frame[3] = 0xFF;
  for (int i = 0; i < 255; ++i) {
    frame[4 + i] = static_cast<uint8_t>(i);
  }
  frame[4 + 255] = Crc8(&frame[2], 257);
  Parser parser;
  Received rx;
  parser.Feed(frame, sizeof(frame), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 1u);
  ASSERT_EQ(rx.frames[0].payload.size(), 255u);
  EXPECT_EQ(rx.frames[0].payload[254], 254);
}

TEST(Parser, ResetClearsPartialFrame)
{
  Parser parser;
  Received rx;
  const auto frame = ChassisVelFrame(1.0f, 2.0f, 3.0f);
  parser.Feed(frame.data(), 6, Received::Handler, &rx);
  parser.Reset();
  parser.Feed(frame.data(), frame.size(), Received::Handler, &rx);
  ASSERT_EQ(rx.frames.size(), 1u);
}

}  // namespace rabcl::protocol

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
