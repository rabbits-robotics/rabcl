// 視線目標: マウスの相対移動を積算して世界系の目標姿勢を保持する（yaw は wrap、pitch はクランプ）
#ifndef RABCL__CONTROLLER__VIEW_TARGET_HPP_
#define RABCL__CONTROLLER__VIEW_TARGET_HPP_

#include <cmath>

#include "rabcl/controller/body_attitude.hpp"

namespace rabcl
{

class ViewTarget
{
public:
  void AddDelta(float dyaw, float dpitch);
  void SetPitchLimits(float min, float max);
  Attitude Get() const;
  void Reset(const Attitude & target);

private:
  Attitude target_ = {0.0f, 0.0f, 0.0f};
  float pitch_min_ = -static_cast<float>(M_PI) / 2.0f;
  float pitch_max_ = static_cast<float>(M_PI) / 2.0f;
};

}  // namespace rabcl

#endif  // RABCL__CONTROLLER__VIEW_TARGET_HPP_
