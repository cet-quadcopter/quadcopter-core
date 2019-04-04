#pragma once

#include <Eigen/Dense>

#include "filters.h"
#include "falcon/utils.h"
#include "falcon/math/calculus.h"


namespace falcon {
namespace state {

struct VelocitySensorParams {
  Eigen::Vector<float, 3> alpha_linear_velocity;
  Eigen::Vector<float, 3> alpha_angular_velocity;
};

class VelocitySensor {
  private:
  ComplementaryFilter<float, 3> linear_velocity_filter_;
  ComplementaryFilter<float, 3> angular_velocity_filter_;
  VelocitySensorParams params_;
  
  falcon::math::Differentiator<float, 3> gyro_diff_;

  public:
  VelocitySensor(VelocitySensorParams params);
  void PostMeasurementInput(
    const Eigen::Vector3f& accelerometer, const Eigen::Vector3f& gyro, 
    const Eigen::Vector3f& gps_velocity, const Eigen::Vector4f& attitude, float dt
  );
  const Eigen::Vector<float, 3>& GetLinearVelocity();
  const Eigen::Vector<float, 3>& GetAngularVelocity();
};

}
}