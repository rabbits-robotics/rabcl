// 首のジンバル: 目標の視線姿勢（世界系）から胴体姿勢を引いて首の関節角を求める
#ifndef RABCL__CONTROLLER__GIMBAL_HPP_
#define RABCL__CONTROLLER__GIMBAL_HPP_

#include "rabcl/controller/body_attitude.hpp"

namespace rabcl
{

class Gimbal
{
public:
  // 各軸 desired - body。yaw は [-pi, pi) に正規化して最短側を返す
  NeckAngles Solve(const Attitude & desired, const Attitude & body) const;
};

}  // namespace rabcl

#endif  // RABCL__CONTROLLER__GIMBAL_HPP_
