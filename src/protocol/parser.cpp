#include "rabcl/protocol/parser.hpp"

#include <cstring>

namespace rabcl::protocol
{

Parser::Parser()
: state_(State::SCAN_H0), pos_(0), body_len_(0), crc_errors_(0), resyncs_(0)
{
}

void Parser::Reset()
{
  state_ = State::SCAN_H0;
  pos_ = 0;
  body_len_ = 0;
}

void Parser::Feed(const uint8_t * data, size_t len, FrameHandler on_frame, void * ctx)
{
  for (size_t i = 0; i < len; ++i) {
    FeedByte(data[i], on_frame, ctx);
  }
}

void Parser::FeedByte(uint8_t byte, FrameHandler on_frame, void * ctx)
{
  switch (state_) {
    case State::SCAN_H0:
      if (byte == HEADER_0) {
        frame_[0] = byte;
        pos_ = 1;
        state_ = State::SCAN_H1;
      }
      break;
    case State::SCAN_H1:
      if (byte == HEADER_1) {
        frame_[1] = byte;
        pos_ = 2;
        state_ = State::TYPE;
      } else {
        ++resyncs_;
        // この byte 自体がヘッダ先頭かもしれない
        if (byte == HEADER_0) {
          pos_ = 1;
        } else {
          pos_ = 0;
          state_ = State::SCAN_H0;
        }
      }
      break;
    case State::TYPE:
      frame_[pos_++] = byte;
      state_ = State::LEN;
      break;
    case State::LEN:
      frame_[pos_++] = byte;
      body_len_ = static_cast<size_t>(byte) + 1;  // payload + crc
      state_ = State::BODY;
      break;
    case State::BODY:
      frame_[pos_++] = byte;
      if (pos_ >= 4 + body_len_) {
        OnFrameComplete(on_frame, ctx);
      }
      break;
  }
}

void Parser::OnFrameComplete(FrameHandler on_frame, void * ctx)
{
  const uint8_t len = frame_[3];
  const uint8_t crc = Crc8(&frame_[2], static_cast<size_t>(len) + 2);
  if (crc == frame_[4 + len]) {
    if (on_frame != nullptr) {
      on_frame(static_cast<Type>(frame_[2]), &frame_[4], len, ctx);
    }
    Reset();
    return;
  }
  // CRC 不一致: ヘッダの次のバイトから探し直す（取り込んだバイト列の中に本物のヘッダがあり得る）
  ++crc_errors_;
  uint8_t pending[MAX_FRAME];
  const size_t pending_len = pos_ - 1;
  std::memcpy(pending, &frame_[1], pending_len);
  Reset();
  Feed(pending, pending_len, on_frame, ctx);
}

}  // namespace rabcl::protocol
