#pragma once

#include "attitude.h"

#include <Eigen/Dense>

#include "../utils.h"
#include "filters.h"

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector4f;

namespace falcon {
namespace state {

struct AttitudeSensorParams {
  float accelerometer_gain;
  float magnetometer_gain;

  Matrix3f covariance_gyro;
  Matrix3f covariance_accelerometer;
  Matrix3f covariance_magnetometer;
};

class AttitudeSensor {
  private:
  KalmanFilter<float, 4, 3, 4> filter_;
  AttitudeSensorParams params_;
  double tm1_;

  public:
  AttitudeSensor(AttitudeSensorParams params);
  void PostControlInput(const Vector3f& gyro, double dt);
  void PostMeasurementInput(const Vector3f& accelerometer, const Vector3f& magnetometer);
  const Vector4f& GetAttitude();
};

}
}