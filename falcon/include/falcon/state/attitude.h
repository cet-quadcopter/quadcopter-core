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

class AttitudeFilter : public KalmanFilter<float, 4, 3, 4> {
  public:
  AttitudeFilter(Vector<float, 4> x_0, Matrix<float, 4, 4> p_0, Matrix<float, 4, 4> c, Matrix<float, 4, 4> h);

  void Predict(
    const Matrix<float, 4, 4>& a, const Vector<float, 3>& u_t, const Matrix<float, 4, 3> b,
    const Vector<float, 4>& w_t, const Matrix<float, 4, 4>& q_t
  ) override;
};

class AttitudeSensor {
  private:
  AttitudeFilter filter_;
  AttitudeSensorParams params_;

  public:
  AttitudeSensor(AttitudeSensorParams params);
  void PostControlInput(const Vector3f& gyro, float dt);
  void PostMeasurementInput(const Vector3f& accelerometer, const Vector3f& magnetometer);
  const Vector4f& GetAttitude() const;
};

}
}