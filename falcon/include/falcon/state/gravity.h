#pragma once

#include <Eigen/Dense>

#include "filters.h"
#include "falcon/utils.h"
#include "falcon/math/calculus.h"


namespace falcon {
namespace state {

struct GravitySensorParams {
  Eigen::Vector<float, 3> alpha;
  float threshold;
};

class GravitySensor {
  private:
  GravitySensorParams params_;
  LowpassFilter<float, 3> filter_;
  math::Differentiator<float, 3> gravity_diff_;

  public:
  GravitySensor(GravitySensorParams params);
  void PostInput(const Eigen::Vector3f& acc, float dt);
  const Eigen::Vector<float, 3>& GetGravity() const;
};

}
}