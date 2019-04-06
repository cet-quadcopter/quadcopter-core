#pragma once

#include <Eigen/Dense>

#include "filters.h"
#include "falcon/utils.h"
#include "falcon/math/calculus.h"


namespace falcon {
namespace state {

class LinearAccelerationSensor {
  private:
  Eigen::Vector3f x_t_;

  public:
  LinearAccelerationSensor();
  void PostInput(const Eigen::Vector3f& a_b, const Eigen::Vector3f& g_n, const Eigen::Vector4f& attitutde);
  const Eigen::Vector<float, 3>& GetLinearAcceleration();
};

}
}