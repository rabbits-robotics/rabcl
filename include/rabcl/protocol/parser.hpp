// 受信パーサ。バイト列を何回に分けて渡してもよく、フレームが揃うたびにハンドラを呼ぶ。
// 動的確保も再帰もなし（内部バッファ 260 B のみ）。ISR / タスク / PC のどこからでも使えるように、
// コールバックは関数ポインタ + ctx。
#ifndef RABCL__PROTOCOL__PARSER_HPP_
#define RABCL__PROTOCOL__PARSER_HPP_

#include <cstddef>
#include <cstdint>

#include "rabcl/protocol/protocol.hpp"

namespace rabcl::protocol
{

class Parser
{
public:
  using FrameHandler = void (*)(Type type, const uint8_t * payload, uint8_t len, void * ctx);

  Parser();

  // data を順に処理し、CRC の合ったフレームごとに on_frame を呼ぶ
  void Feed(const uint8_t * data, size_t len, FrameHandler on_frame, void * ctx);
  void Reset();

  uint32_t crc_errors() const {return crc_errors_;}
  uint32_t resyncs() const {return resyncs_;}  // ヘッダ探索に戻った回数（CRC 不一致を除く）

private:
  enum class State : uint8_t
  {
    SCAN_H0,
    SCAN_H1,
    TYPE,
    LEN,
    BODY,  // payload + crc
  };

  // 1 バイト進める。フレームが揃ったら true
  bool Push(uint8_t byte);
  // frame_[0..pos_) に揃ったフレームを処理する。CRC 不一致なら取り込んだバイト列を
  // 1 バイトずらして同じバッファ内で走査し直す（コピーも再帰もしない）
  void Complete(FrameHandler on_frame, void * ctx);

  State state_;
  uint8_t frame_[MAX_FRAME];
  size_t pos_;
  size_t body_len_;  // payload + crc のバイト数
  uint32_t crc_errors_;
  uint32_t resyncs_;
};

}  // namespace rabcl::protocol

#endif  // RABCL__PROTOCOL__PARSER_HPP_
