// 装填のジャム検知と自動復旧の状態機械。100 ms ごとのサーボの読み（残りステップ・負荷・電流・ERR）と時刻を入れると、
// 「出させない向き」「送る相対 Move」「再接続が要るか」を返す。サーボには触れない。
// 戻す量も進める量も 1 セルの整数倍なので、トルクを切らない限りサーボの目標は穴の上に残る（位相が保たれる）。
// NORMAL → 詰まり → BACKING（k セル戻す）→ RETRYING（k セル進め直す）→ NORMAL。失敗が retry_limit 回で JAMMED（逆転で解除）。
// 戻しが効かないとき（戻しのタイムアウト、JAMMED で戻し切れない）と ERR に過負荷保護（0x20）が立ったときは RECONNECT
// （トルクを切って位相合わせからやり直す。Reset で戻す）
#ifndef RABCL__CONTROLLER__JAM_RECOVERY_HPP_
#define RABCL__CONTROLLER__JAM_RECOVERY_HPP_

#include <cstdint>

namespace rabcl
{

struct JamRecoveryConfig
{
  float counts_per_cell;      // サーボのカウント / 1 セル
  int16_t load_limit;         // 0.1 %。|負荷| がこれ以上で詰まりの候補
  int16_t current_limit;      // 6.5 mA 単位。|電流| がこれ以上で詰まりの候補
  uint8_t stall_reads;        // 連続でこの回数の読みで詰まりとみなす
  uint8_t queue_limit_cells;  // |残り| がこのセル数を超えたら同じ向きのステップを出させない
  uint8_t retry_limit;        // 進め直しの失敗がこの回数で JAMMED
  float timeout_s;            // 戻し・進め直しの移動が到達するまでの待ち [s]
  int16_t settle_counts;      // |残り| がこれ以下なら到達（静止時の残りは 0 にならず数カウント残る）
};

class JamRecovery
{
public:
  enum class State : uint8_t
  {
    NORMAL = 0,
    BACKING = 1,    // k セル戻している
    RETRYING = 2,   // k セル進め直している
    JAMMED = 3,     // あきらめた。正転は出させない、逆転で NORMAL
    RECONNECT = 4,  // 過負荷保護。トルクを切って再接続してから Reset
  };

  struct Action
  {
    bool block_forward;  // 正転のステップを出させない（Feeder に渡すモードを STOP にする）
    bool block_reverse;  // 逆転のステップを出させない
    int16_t move;        // 0 以外ならこの相対ステップを送る（復旧の戻し / 進め直し）
    bool reconnect;      // トルクを切って再接続する
  };

  static constexpr uint8_t ERR_OVERLOAD = 0x20;  // ST3215 の ERR: 過負荷保護

  explicit JamRecovery(const JamRecoveryConfig & config);

  // サーボの読みごとに呼ぶ。remaining はステップモードの現在位置（目標までの残り、正転で正）。
  // error は読みの直後の ERR
  Action Observe(int16_t remaining, int16_t load, int16_t current, uint8_t error, float now_s);
  // 逆転のステップを送ったら呼ぶ（JAMMED の解除、復旧中の中断。操作者優先）
  void NotifyReverse();
  // 通信途絶からの再接続が通ったら呼ぶ。NORMAL、試行回数 0
  void Reset();
  // ジャム起因（RECONNECT）の再接続が通ったら呼ぶ。再接続も試行に数え、retry_limit 回目なら JAMMED、それ以外は NORMAL。
  // 試行回数は詰まらずに 1 セル送れたとき（または操作者の逆転）に 0 に戻る。now_s は JAMMED の到達待ちの起点
  void ResumeAfterReconnect(float now_s);

  State state() const {return state_;}
  uint8_t tries() const {return tries_;}  // 進め直しの失敗 + ジャム起因の再接続の回数
  uint32_t recoveries() const {return recoveries_;}  // NORMAL に戻れた回数
  uint32_t jams() const {return jams_;}              // JAMMED になった回数

private:
  bool Stalled(int16_t remaining, int16_t load, int16_t current);
  bool Arrived(int16_t remaining) const;
  int16_t Limit() const;

  JamRecoveryConfig config_;
  State state_ = State::NORMAL;
  uint8_t stall_count_ = 0;
  uint8_t tries_ = 0;
  int16_t back_counts_ = 0;
  float started_s_ = 0.0f;
  bool cell_in_flight_ = false;  // NORMAL で残りが動いている（1 セル送った）のを見た
  uint32_t recoveries_ = 0;
  uint32_t jams_ = 0;
};

}  // namespace rabcl

#endif  // RABCL__CONTROLLER__JAM_RECOVERY_HPP_
