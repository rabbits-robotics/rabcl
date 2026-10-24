// 胴体姿勢の推定: IMU は首の先（カメラ link）にあるので、IMU 姿勢から首の関節角を引いて胴体姿勢を得る。
// IMU 更新の合間はシャシの yaw 速度で yaw のみ積分して補う。
#ifndef RABCL__CONTROLLER__BODY_ATTITUDE_HPP_
#define RABCL__CONTROLLER__BODY_ATTITUDE_HPP_

namespace rabcl
{

// [rad]、REP-103（yaw: 左が正、pitch: 前が下がると正、roll: 右が下がると正）
struct Attitude
{
  float yaw;
  float pitch;
  float roll;
};

// 首の関節角 [rad]（yaw、pitch、roll の順に胴体側から）
struct NeckAngles
{
  float yaw;
  float pitch;
  float roll;
};

class BodyAttitude
{
public:
  // IMU 姿勢と、その計測時刻の首関節角から胴体姿勢を更新する（yaw は AnchorImu のオフセットを加える）
  void UpdateFromImu(const Attitude & imu, const NeckAngles & joint_at_imu_time);
  // IMU が使えるようになった（または再初期化された）とき: IMU の yaw 基準（モード投入時 = 0）を
  // 今の推定に合わせて繋ぐ。呼んだ直後の Get().yaw は変わらず、以後は IMU の変化分だけ追う
  void AnchorImu(const Attitude & imu, const NeckAngles & joint_at_imu_time);
  // シャシの yaw 速度 wz [rad/s] を dt [s] だけ積分する（yaw のみ）
  void UpdateFromChassisRate(float wz, float dt);
  Attitude Get() const;
  void Reset();

private:
  Attitude body_ = {0.0f, 0.0f, 0.0f};
  float imu_yaw_offset_ = 0.0f;  // IMU の yaw に足して胴体の基準に合わせる量
};

}  // namespace rabcl

#endif  // RABCL__CONTROLLER__BODY_ATTITUDE_HPP_
