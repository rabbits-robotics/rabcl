// 装填のステップ指令: モード（停止 / 正転 / 逆転）と時刻から「今サーボへ送る相対カウント」を決める。
// 1 セルのカウントが非整数（gamma: 4096 / 8 セル / 増速 3 = 170.67）でも端数が溜まらないよう、
// 目標カウントを実数で累積し round の差分を送る（171, 170, 171, …）。
// 押下エッジで 1 セルを保留し、前のステップから period_s 経っていれば出す。押し続けている間は period_s ごとに 1 セル。
// 保留はモードが STOP に戻っても残る（短く叩いても取りこぼさない）。途絶のときは Cancel で捨てる
#ifndef RABCL__CONTROLLER__FEEDER_HPP_
#define RABCL__CONTROLLER__FEEDER_HPP_

#include <cstdint>

namespace rabcl
{

struct FeederConfig
{
  float counts_per_cell;  // サーボのカウント / 1 セル
  float period_s;         // 1 セルの最短間隔 [s]
};

class Feeder
{
public:
  enum class Mode : uint8_t
  {
    STOP = 0,
    FORWARD = 1,
    REVERSE = 2,
  };

  explicit Feeder(const FeederConfig & config);

  // 毎 tick 呼ぶ。今送る相対ステップ数（0 = 送らない）。逆転は負
  int16_t Update(Mode mode, float now_s);
  void Cancel();  // 保留を捨てる（途絶）

  float target_counts() const {return target_;}
  int32_t sent_counts() const {return sent_;}
  int32_t cells() const {return cells_;}

private:
  static constexpr float PERIOD_TOLERANCE_S = 0.001f;  // float の時刻差の誤差吸収（tick 10 ms より十分小さい）

  FeederConfig config_;
  Mode last_mode_ = Mode::STOP;
  Mode pending_mode_ = Mode::STOP;
  bool pending_ = false;
  bool stepped_ = false;
  float last_step_s_ = 0.0f;
  float target_ = 0.0f;
  int32_t sent_ = 0;
  int32_t cells_ = 0;
};

}  // namespace rabcl

#endif  // RABCL__CONTROLLER__FEEDER_HPP_
