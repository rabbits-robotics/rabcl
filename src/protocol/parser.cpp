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
    if (Push(data[i])) {
      Complete(on_frame, ctx);
    }
  }
}

bool Parser::Push(uint8_t byte)
{
  switch (state_) {
    case State::SCAN_H0:
      if (byte == HEADER_0) {
        frame_[0] = byte;
        pos_ = 1;
        state_ = State::SCAN_H1;
      }
      return false;
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
      return false;
    case State::TYPE:
      frame_[pos_++] = byte;
      state_ = State::LEN;
      return false;
    case State::LEN:
      frame_[pos_++] = byte;
      body_len_ = static_cast<size_t>(byte) + 1;  // payload + crc
      state_ = State::BODY;
      return false;
    case State::BODY:
      frame_[pos_++] = byte;
      return pos_ >= 4 + body_len_;
  }
  return false;
}

void Parser::Complete(FrameHandler on_frame, void * ctx)
{
  // frame_[0..pos_) が揃ったフレーム。avail はバッファ内の有効バイト数（再走査中は pos_ より長い）
  size_t avail = pos_;
  while (true) {
    const uint8_t len = frame_[3];
    const uint8_t crc = Crc8(&frame_[2], static_cast<size_t>(len) + 2);
    size_t rest_begin;
    if (crc == frame_[4 + len]) {
      if (on_frame != nullptr) {
        on_frame(static_cast<Type>(frame_[2]), &frame_[4], len, ctx);
      }
      rest_begin = pos_;  // フレームの後ろに残っているバイト列
    } else {
      ++crc_errors_;
      rest_begin = 1;  // ヘッダの次のバイトから探し直す（取り込んだ中に本物のヘッダがあり得る）
    }
    Reset();
    const size_t n = avail - rest_begin;
    if (n == 0) {
      return;
    }
    // 残りを先頭に詰めて、同じバッファ内で走査し直す。
    // Push は frame_[pos_] に書き、pos_ は読み出し位置 i を超えないので上書きは起きない
    std::memmove(frame_, &frame_[rest_begin], n);
    avail = n;
    bool complete = false;
    for (size_t i = 0; i < avail; ++i) {
      if (Push(frame_[i])) {
        complete = true;
        // frame_[i + 1 .. avail) はまだ読んでいないバイト列。次の周回で rest_begin = pos_ から扱う
        std::memmove(&frame_[pos_], &frame_[i + 1], avail - (i + 1));
        avail = pos_ + (avail - (i + 1));
        break;
      }
    }
    if (!complete) {
      return;  // 途中までのフレームは frame_[0..pos_) に保持したまま、次の Feed を待つ
    }
  }
}

}  // namespace rabcl::protocol
