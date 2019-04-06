#pragma once

#include <Eigen/Dense>

#include "filters.h"
#include "falcon/utils.h"
#include "falcon/math/calculus.h"


namespace falcon {
namespace state {

struct AngularVelocitySensorParams {
  Eigen::Vector<float, 3> alpha;
};

class AngularVelocitySensor {
  private:
  ComplementaryFilter<float, 3> filter_;
  AngularVelocitySensorParams params_;
  
  falcon::math::Differentiator<float, 3> gyro_diff_;

  public:
  AngularVelocitySensor(AngularVelocitySensorParams params);
  void PostInput(const Eigen::Vector3f& gyro, const Eigen::Vector4f& attitude, float dt);
  const Eigen::Vector<float, 3>& GetAngularVelocity() const;
};

}
}