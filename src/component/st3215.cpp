#include "rabcl/component/st3215.hpp"

#include <cstring>

namespace rabcl
{

namespace
{
constexpr uint8_t HEADER = 0xFF;
constexpr uint8_t STATUS_PACKET_LEN_MIN = 2;  // ERR + CHK
}  // namespace

size_t St3215Protocol::BuildPing(uint8_t id, uint8_t * out)
{
  out[0] = HEADER;
  out[1] = HEADER;
  out[2] = id;
  out[3] = 2;
  out[4] = INST_PING;
  out[5] = static_cast<uint8_t>(~(id + 2 + INST_PING));
  return 6;
}

size_t St3215Protocol::BuildRead(uint8_t id, uint8_t reg, uint8_t len, uint8_t * out)
{
  out[0] = HEADER;
  out[1] = HEADER;
  out[2] = id;
  out[3] = 4;
  out[4] = INST_READ;
  out[5] = reg;
  out[6] = len;
  out[7] = static_cast<uint8_t>(~(id + 4 + INST_READ + reg + len));
  return 8;
}

size_t St3215Protocol::BuildWrite(
  uint8_t id, uint8_t reg, const uint8_t * data, uint8_t len, uint8_t * out)
{
  const uint8_t packet_len = static_cast<uint8_t>(len + 3);
  out[0] = HEADER;
  out[1] = HEADER;
  out[2] = id;
  out[3] = packet_len;
  out[4] = INST_WRITE;
  out[5] = reg;
  uint8_t sum = static_cast<uint8_t>(id + packet_len + INST_WRITE + reg);
  for (uint8_t i = 0; i < len; ++i) {
    out[6 + i] = data[i];
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  out[6 + len] = static_cast<uint8_t>(~sum);
  return 7 + len;
}

uint16_t St3215Protocol::EncodeSigned(int16_t v)
{
  if (v < 0) {
    return static_cast<uint16_t>(static_cast<uint16_t>(-v) | 0x8000);
  }
  return static_cast<uint16_t>(v);
}

int16_t St3215Protocol::DecodeSigned(uint16_t raw)
{
  const int16_t mag = static_cast<int16_t>(raw & 0x7FFF);
  return (raw & 0x8000) ? static_cast<int16_t>(-mag) : mag;
}

int16_t St3215Protocol::DecodeLoad(uint16_t raw)
{
  const int16_t mag = static_cast<int16_t>(raw & 0x03FF);
  return (raw & 0x0400) ? static_cast<int16_t>(-mag) : mag;
}

void St3215Protocol::PutU16(uint16_t v, uint8_t * out)
{
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>(v >> 8);
}

uint16_t St3215Protocol::GetU16(const uint8_t * in)
{
  return static_cast<uint16_t>(in[0] | (static_cast<uint16_t>(in[1]) << 8));
}

bool St3215Protocol::ParseResponse(
  const uint8_t * buf, size_t n, Response & out, size_t & consumed)
{
  // FF FF を探す。FF が 3 つ以上続いたら最後の 2 つを取る（ごみの FF を飛ばす）
  size_t start = 0;
  bool found = false;
  for (size_t i = 0; i + 1 < n && i < HEADER_SEARCH; ++i) {
    if (buf[i] == HEADER && buf[i + 1] == HEADER) {
      if (i + 2 < n && buf[i + 2] == HEADER) {
        continue;
      }
      start = i;
      found = true;
      break;
    }
  }
  if (!found || start + 4 > n) {
    return false;
  }
  const uint8_t id = buf[start + 2];
  const uint8_t len = buf[start + 3];
  if (len < STATUS_PACKET_LEN_MIN || static_cast<size_t>(len - STATUS_PACKET_LEN_MIN) > MAX_DATA) {
    return false;
  }
  const size_t total = start + 4 + len;  // ヘッダ 2 + ID + LEN + (ERR + DATA + CHK)
  if (total > n) {
    return false;
  }
  const uint8_t error = buf[start + 4];
  const uint8_t data_len = static_cast<uint8_t>(len - STATUS_PACKET_LEN_MIN);
  uint8_t sum = static_cast<uint8_t>(id + len + error);
  for (uint8_t i = 0; i < data_len; ++i) {
    sum = static_cast<uint8_t>(sum + buf[start + 5 + i]);
  }
  if (static_cast<uint8_t>(~sum) != buf[total - 1]) {
    return false;
  }
  out.id = id;
  out.error = error;
  out.len = data_len;
  std::memcpy(out.data, buf + start + 5, data_len);
  consumed = total;
  return true;
}

St3215::St3215(St3215Port & port, uint8_t id, uint32_t timeout_ms)
: port_(port), id_(id), timeout_ms_(timeout_ms)
{
}

bool St3215::Exchange(
  const uint8_t * tx, size_t tx_len, uint8_t expect, St3215Protocol::Response & resp)
{
  uint8_t rx[St3215Protocol::MAX_PACKET];
  const size_t want = St3215Protocol::ResponseLength(expect);
  const int got = port_.Transfer(tx, tx_len, rx, want, timeout_ms_);
  if (got < static_cast<int>(want)) {
    ++timeouts_;
    return false;
  }
  size_t consumed = 0;
  if (!St3215Protocol::ParseResponse(rx, static_cast<size_t>(got), resp, consumed)) {
    return false;
  }
  last_error_ = resp.error;
  return true;
}

bool St3215::Ping()
{
  uint8_t tx[St3215Protocol::MAX_PACKET];
  const size_t n = St3215Protocol::BuildPing(id_, tx);
  St3215Protocol::Response resp;
  return Exchange(tx, n, 0, resp);
}

bool St3215::Read(uint8_t reg, uint8_t * data, uint8_t len)
{
  if (len > St3215Protocol::MAX_DATA) {
    return false;
  }
  uint8_t tx[St3215Protocol::MAX_PACKET];
  const size_t n = St3215Protocol::BuildRead(id_, reg, len, tx);
  St3215Protocol::Response resp;
  if (!Exchange(tx, n, len, resp) || resp.len != len) {
    return false;
  }
  std::memcpy(data, resp.data, len);
  return true;
}

bool St3215::ReadU8(uint8_t reg, uint8_t & v)
{
  return Read(reg, &v, 1);
}

bool St3215::ReadU16(uint8_t reg, uint16_t & v)
{
  uint8_t b[2];
  if (!Read(reg, b, 2)) {
    return false;
  }
  v = St3215Protocol::GetU16(b);
  return true;
}

bool St3215::ReadS16(uint8_t reg, int16_t & v)
{
  uint16_t raw;
  if (!ReadU16(reg, raw)) {
    return false;
  }
  v = St3215Protocol::DecodeSigned(raw);
  return true;
}

bool St3215::Write(uint8_t reg, const uint8_t * data, uint8_t len)
{
  if (len > St3215Protocol::MAX_DATA) {
    return false;
  }
  uint8_t tx[St3215Protocol::MAX_PACKET];
  const size_t n = St3215Protocol::BuildWrite(id_, reg, data, len, tx);
  St3215Protocol::Response resp;
  return Exchange(tx, n, 0, resp);
}

bool St3215::WriteU8(uint8_t reg, uint8_t v)
{
  return Write(reg, &v, 1);
}

bool St3215::WriteU16(uint8_t reg, uint16_t v)
{
  uint8_t b[2];
  St3215Protocol::PutU16(v, b);
  return Write(reg, b, 2);
}

bool St3215::SetMode(St3215Protocol::Mode mode)
{
  if (!WriteU8(St3215Protocol::REG_LOCK, 0)) {
    return false;
  }
  const bool ok = WriteU8(St3215Protocol::REG_MODE, static_cast<uint8_t>(mode));
  const bool locked = WriteU8(St3215Protocol::REG_LOCK, 1);
  return ok && locked;
}

bool St3215::SetTorque(bool on)
{
  return WriteU8(St3215Protocol::REG_TORQUE_ENABLE, on ? 1 : 0);
}

bool St3215::Move(int16_t position_or_steps, uint16_t speed, uint8_t acc)
{
  uint8_t b[7];
  b[0] = acc;
  St3215Protocol::PutU16(St3215Protocol::EncodeSigned(position_or_steps), b + 1);
  St3215Protocol::PutU16(0, b + 3);  // 時間は使わない
  St3215Protocol::PutU16(speed, b + 5);
  return Write(St3215Protocol::REG_ACC, b, 7);
}

bool St3215::ReadStatus(St3215Status & out)
{
  uint8_t d[15];
  if (!Read(St3215Protocol::REG_PRESENT_POSITION, d, 15)) {
    return false;
  }
  out.position = St3215Protocol::DecodeSigned(St3215Protocol::GetU16(d + 0));
  out.speed = St3215Protocol::DecodeSigned(St3215Protocol::GetU16(d + 2));
  out.load = St3215Protocol::DecodeLoad(St3215Protocol::GetU16(d + 4));
  out.voltage = d[6];
  out.temperature = d[7];
  out.moving = d[10] != 0;
  out.current = St3215Protocol::DecodeSigned(St3215Protocol::GetU16(d + 13));
  return true;
}

}  // namespace rabcl
