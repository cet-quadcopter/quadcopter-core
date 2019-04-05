#pragma once

#include <Eigen/Dense>

#include "filters.h"
#include "falcon/utils.h"
#include "falcon/math/calculus.h"


namespace falcon {
namespace state {

struct LinearVelocitySensorParams {
  Eigen::Matrix3f covariance_accelerometer;
  Eigen::Matrix3f covariance_gps_velocity;
};

class LinearVelocitySensor {
  private:
  LinearVelocitySensorParams params_;
  KalmanFilter<float, 3, 3, 3> filter_;
  
  public:
  LinearVelocitySensor(LinearVelocitySensorParams params);
  void PostControlInput(const Eigen::Vector3f& accelerometer, const Eigen::Vector4f& attitude, float dt);
  void PostMeasurementInput(const Eigen::Vector3f& gps_velocity);
  const Eigen::Vector<float, 3>& GetLinearVelocity();
};

}
}