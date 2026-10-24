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


}  // namespace rabcl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
